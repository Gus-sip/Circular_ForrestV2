// =============================================================================
// MODEM NB-IoT + MQTT (Quectel BC660K-GL) - IMPLEMENTACION
// =============================================================================
//
// Ver la descripcion general en modem_nbiot_mqtt.h. Orden de este archivo:
//   1. Arranque, tick() y configuracion en marcha.
//   2. Anillo de lecturas pendientes.
//   3. Motor de comandos AT: enviar, leer lineas, avisos asincronos (URC).
//   4. Ordenes recibidas de ThingsBoard (RPC) y su reenvio a los nodos.
//   5. Maquina de estados: encendido, configuracion, registro, conexion MQTT,
//      espera, publicacion, respuestas y error/recuperacion.
//   6. Construccion del JSON de telemetria para la pasarela.

#include "modem_nbiot_mqtt.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

ModemNBIoTMqtt::ModemNBIoTMqtt(HardwareSerial &serial, uint8_t rxPin, uint8_t txPin, uint8_t enPin,
                                uint8_t channelPin, uint8_t pwrkeyPin, uint32_t baud)
    : _serial(serial),
      _rxPin(rxPin),
      _txPin(txPin),
      _enPin(enPin),
      _channelPin(channelPin),
      _pwrkeyPin(pwrkeyPin),
      _baud(baud) {}

// Pines en su estado de reposo: modem sin alimentacion y canal cerrado.
void ModemNBIoTMqtt::begin() {
  pinMode(_enPin, OUTPUT);
  digitalWrite(_enPin, NBIOT_DISABLE);
  pinMode(_channelPin, OUTPUT);
  digitalWrite(_channelPin, !NBIOT_CHANNEL_ACTIVE);
  pinMode(_pwrkeyPin, OUTPUT);
  digitalWrite(_pwrkeyPin, !NBIOT_PWRKEY_ACTIVE);  // en reposo hasta que POWERING lo pulse
  setState(State::OFF);
}

// Una vuelta de trabajo: leer lo que haya llegado del modem, vigilar el plazo del
// comando en curso, avanzar el estado actual y vigilar los plazos de las ordenes a
// los nodos.
void ModemNBIoTMqtt::tick() {
  pumpSerial();
  checkCommandTimeout();

  switch (_state) {
    case State::OFF: tickOff(); break;
    case State::POWERING: tickPowering(); break;
    case State::WAIT_AT: tickWaitAt(); break;
    case State::CONFIG: tickConfig(); break;
    case State::ATTACHING: tickAttaching(); break;
    case State::MQTT_CONNECT: tickMqttConnect(); break;
    case State::IDLE: tickIdle(); break;
    case State::PUBLISHING: tickPublishing(); break;
    case State::RPC_REPLY: tickRpcReply(); break;
    case State::ERROR: tickError(); break;
  }

  tickNodeCommandTimeout();
}

// ---------- Configuracion en marcha, ajustable desde Modulo A ----------

uint32_t ModemNBIoTMqtt::setBatchSeconds(uint32_t seconds) {
  if (seconds < MQTT_BATCH_SECONDS_MIN) seconds = MQTT_BATCH_SECONDS_MIN;
  if (seconds > MQTT_BATCH_SECONDS_MAX) seconds = MQTT_BATCH_SECONDS_MAX;
  _batchSecondsTarget = seconds;
  Serial.printf("[nbiot-mqtt] uplink period set to %lus\n", (unsigned long)seconds);
  return seconds;
}

uint16_t ModemNBIoTMqtt::setBatchReadings(uint16_t readings) {
  if (readings < MQTT_BATCH_READINGS_MIN) readings = MQTT_BATCH_READINGS_MIN;
  if (readings > MQTT_BATCH_READINGS_MAX) readings = MQTT_BATCH_READINGS_MAX;
  _batchReadingsTarget = readings;
  Serial.printf("[nbiot-mqtt] uplink batch size set to %u readings\n", readings);
  return readings;
}

// ---------- Anillo de lecturas pendientes de subir ----------

// Mete una lectura al final del anillo. Si esta lleno, se descarta la mas antigua.
void ModemNBIoTMqtt::enqueue(const SensorSnapshot &snap) {
  if (_ringCount == 0) _oldestPendingMs = snap.lastHeardMs ? snap.lastHeardMs : millis();

  if (_ringCount >= NBIOT_RING_CAPACITY) {
    _ringHead = (uint8_t)((_ringHead + 1) % NBIOT_RING_CAPACITY);
    _ringCount--;
    _packetsDropped++;
    _oldestPendingMs = ringPeek(0).lastHeardMs;
  }

  uint8_t idx = (uint8_t)((_ringHead + _ringCount) % NBIOT_RING_CAPACITY);
  _ring[idx] = snap;
  _ringCount++;
}

// La lectura numero i contando desde la mas antigua.
const SensorSnapshot &ModemNBIoTMqtt::ringPeek(uint8_t i) const {
  return _ring[(_ringHead + i) % NBIOT_RING_CAPACITY];
}

// Quita del anillo las lecturas que acaba de publicar la ultima publicacion.
void ModemNBIoTMqtt::popSentReadings() {
  uint8_t n = _lastPublishCount;
  if (n > _ringCount) n = _ringCount;
  _ringHead = (uint8_t)((_ringHead + n) % NBIOT_RING_CAPACITY);
  _ringCount = (uint8_t)(_ringCount - n);
  if (_ringCount > 0) _oldestPendingMs = ringPeek(0).lastHeardMs;
}

// ---------- El unico camino de los comandos AT ----------

// Manda un comando AT y lo deja "en curso". No espera: la respuesta la recogen
// pumpSerial()/handleLine() en ticks posteriores, y el estado la consulta con
// consumeOutcome().
void ModemNBIoTMqtt::issueCommand(const char *cmd, uint32_t timeoutMs, CmdKind kind, const uint8_t *sendPayload,
                                   size_t sendPayloadLen) {
  _cmd = PendingCmd{};
  _cmd.active = true;
  _cmd.kind = kind;
  _cmd.sentAtMs = millis();
  _cmd.timeoutMs = timeoutMs;
  _cmd.sendPayload = sendPayload;
  _cmd.sendPayloadLen = sendPayloadLen;
  _cmd.awaitingPrompt = (kind == CmdKind::QMTPUB);
  Serial.print("[nbiot-mqtt] -> ");
  Serial.println(cmd);
  _serial.print(cmd);
  _serial.print("\r\n");
}

// Lee los bytes que haya mandado el modem y los junta en lineas.
void ModemNBIoTMqtt::pumpSerial() {
  while (_serial.available()) {
    char c = (char)_serial.read();

    // AT+QMTPUB contesta con un '>' suelto (sin fin de linea) pidiendo el JSON en
    // crudo; se atrapa aqui, antes de juntar lineas, porque no es una linea. El
    // contenido se termina con Ctrl+Z (0x1A), no con una cabecera de longitud, lo
    // que evita tener que escapar las comillas del JSON en la linea de comando.
    if (_cmd.active && _cmd.kind == CmdKind::QMTPUB && _cmd.awaitingPrompt && c == '>') {
      Serial.print("[nbiot-mqtt] payload (");
      Serial.print((unsigned)_cmd.sendPayloadLen);
      Serial.print(" bytes): ");
      Serial.write(_cmd.sendPayload, _cmd.sendPayloadLen);
      Serial.println();
      _serial.write(_cmd.sendPayload, _cmd.sendPayloadLen);
      _serial.write((uint8_t)0x1A);
      _cmd.awaitingPrompt = false;
      _lineLen = 0;
      continue;
    }

    if (c == '\n') {
      if (_lineOverflowed) {
        // Decirlo. El codigo anterior volvia a empezar el buffer a mitad de linea y
        // trataba el RESTO como una linea completa, lo que convertia un mensaje
        // demasiado largo en un fragmento con apariencia razonable y escondia el
        // fallo real.
        Serial.printf("[nbiot-mqtt] URC LONGER THAN %u BYTES - DISCARDED. A command "
                      "may have been lost; raise kLineBufLen\n",
                      (unsigned)kLineBufLen);
        _lineOverflowed = false;
      } else if (_lineLen > 0) {
        _lineBuf[_lineLen] = '\0';
        handleLine(_lineBuf, _lineLen);
      }
      _lineLen = 0;
    } else if (c != '\r') {
      if (_lineLen < sizeof(_lineBuf) - 1) {
        _lineBuf[_lineLen++] = c;
      } else {
        // Descartar el RESTO de esta linea en vez de empezar una nueva dentro de
        // ella. Una linea a medias nunca es una entrada valida, y fingir lo contrario
        // es lo que pasaba colas de JSON al interprete.
        _lineOverflowed = true;
      }
    }
  }
}

// Decide que es cada linea: un aviso espontaneo, o parte de la respuesta del
// comando en curso (OK, ERROR, el aviso asincrono de un QMT*, o una linea de datos).
void ModemNBIoTMqtt::handleLine(const char *line, size_t len) {
  // UNA ORDEN ENTRANTE ES ESPONTANEA POR DEFINICION, ASI QUE SE ATIENDE ANTES QUE
  // NADA.
  //
  // Antes estaba debajo del reparto por comando, asi que un +QMTRECV que llegaba con
  // un comando en curso solo se salvaba para los cuatro comandos cuyo resultado real
  // llega como aviso asincrono (QMTOPEN/QMTCONN/QMTPUB/QMTSUB). Con cualquier otro
  // comando - AT+CSQ, AT+QMTCFG, un QMTPUB que aun esperaba su OK - caia al final de
  // esta funcion y se guardaba como linea de datos del comando, es decir, se perdia
  // en silencio.
  //
  // Visto en vivo el 2026-10-05: una orden se entrego en el instante de suscribirse
  // (la sesion persistente funcionando como debe - el servidor la tenia en cola) y se
  // la trago el propio intercambio de AT+QMTSUB. Aparecia en el log como
  // "<- +QMTRECV: ..." en vez de "downlink on ...", que es la pista: el prefijo "<- "
  // significa que un comando se la quedo.
  //
  // El modem intercala avisos con respuestas de comandos cuando le parece, asi que no
  // hay ningun estado en el que ignorar uno sea correcto.
  if (strncmp(line, "+QMTRECV", 8) == 0) {
    handleUrc(line, len);
    return;
  }

  if (!_cmd.active) {
    handleUrc(line, len);
    return;
  }
  Serial.print("[nbiot-mqtt] <- ");
  Serial.println(line);

  if (strcmp(line, "OK") == 0) {
    if (_cmd.kind == CmdKind::QMTOPEN || _cmd.kind == CmdKind::QMTCONN || _cmd.kind == CmdKind::QMTPUB ||
        _cmd.kind == CmdKind::QMTSUB) {
      _cmd.sawOk = true;  // confirmacion recibida: el resultado real es el aviso asincrono de abajo
      return;
    }
    completeCommand(CmdOutcome::OK);
    return;
  }
  if (strncmp(line, "ERROR", 5) == 0 || strncmp(line, "+CME ERROR", 10) == 0 ||
      strncmp(line, "+CMS ERROR", 10) == 0) {
    completeCommand(CmdOutcome::ERR);
    return;
  }

  if (_cmd.kind == CmdKind::QMTOPEN && _cmd.sawOk) {
    int idx, result;
    if (NbiotProtocol::parseQmtopen(line, idx, result)) {
      completeCommand(result == 0 ? CmdOutcome::OK : CmdOutcome::ERR);
      return;
    }
    handleUrc(line, len);  // aviso ajeno intercalado: este comando sigue pendiente
    return;
  }
  if (_cmd.kind == CmdKind::QMTCONN && _cmd.sawOk) {
    int idx, result, retCode;
    if (NbiotProtocol::parseQmtconn(line, idx, result, retCode)) {
      completeCommand((result == 0 && retCode == 0) ? CmdOutcome::OK : CmdOutcome::ERR);
      return;
    }
    handleUrc(line, len);
    return;
  }
  if (_cmd.kind == CmdKind::QMTPUB && _cmd.sawOk) {
    int idx, msgId, result;
    if (NbiotProtocol::parseQmtpub(line, idx, msgId, result)) {
      completeCommand(result == 0 ? CmdOutcome::OK : CmdOutcome::ERR);
      return;
    }
    handleUrc(line, len);
    return;
  }
  if (_cmd.kind == CmdKind::QMTSUB && _cmd.sawOk) {
    int idx, msgId, result;
    if (NbiotProtocol::parseQmtsub(line, idx, msgId, result)) {
      completeCommand(result == 0 ? CmdOutcome::OK : CmdOutcome::ERR);
      return;
    }
    handleUrc(line, len);
    return;
  }

  // Cualquier otra linea con un comando simple en curso es su linea de datos (p. ej.
  // "+CPIN: READY" o "+CEREG: 1,5" antes del OK).
  strncpy(_cmd.infoLine, line, sizeof(_cmd.infoLine) - 1);
  _cmd.hasInfoLine = true;
}

// Avisos espontaneos del modem: sesion MQTT caida, orden entrante, arranque.
void ModemNBIoTMqtt::handleUrc(const char *line, size_t len) {
  (void)len;

  int idx, errCode;
  if (NbiotProtocol::parseQmtstat(line, idx, errCode)) {
    if (idx == MQTT_CLIENT_IDX) {
      _mqttConnected = false;
      Serial.printf("[nbiot-mqtt] +QMTSTAT fired (err=%d) - session marked dead, "
                    "will reconnect from IDLE\n",
                    errCode);
    }
    return;
  }

  // Mensaje MQTT entrante - el camino de las ordenes desde Modulo A. Llega como aviso
  // espontaneo en cualquier momento, incluso en mitad de la respuesta de otro
  // comando, y por eso se atiende aqui y no en una rama de comando.
  NbiotProtocol::MqttMessage inbound;
  if (NbiotProtocol::parseQmtrecv(line, inbound)) {
    _downlinksReceived++;
    Serial.printf("[nbiot-mqtt] downlink on \"%s\": %s\n", inbound.topic, inbound.payload);
    handleInboundMqtt(inbound);
    return;
  }

  if (strstr(line, "RDY") || strstr(line, "QNBIOTEVENT")) {
    _sawBootUrc = true;
    return;
  }

  Serial.printf("[nbiot-mqtt] urc: %s\n", line);
}

// ---------- Ordenes entrantes: Modulo A -> Modulo B (-> Modulo C) ----------

// Reparte una orden entrante segun su topic.
void ModemNBIoTMqtt::handleInboundMqtt(const NbiotProtocol::MqttMessage &msg) {
  // El topic de RPC propio de la pasarela lleva el id de la peticion en el propio
  // topic:  v1/devices/me/rpc/request/<id>
  if (strncmp(msg.topic, "v1/devices/me/rpc/request/", 26) == 0) {
    handleOwnRpc(msg);
    return;
  }
  if (strcmp(msg.topic, MQTT_TOPIC_GATEWAY_RPC) == 0) {
    handleGatewayRpc(msg);
    return;
  }
  Serial.printf("[nbiot-mqtt] downlink on an unhandled topic, ignored: %s\n", msg.topic);
}

// RPC dirigido al propio dispositivo pasarela = reconfigurar Modulo B, o la orden
// configurarModuloC del protocolo A->C.
//   {"method":"cfg","params":{"UPLINK":300,"BATCH":6}}
void ModemNBIoTMqtt::handleOwnRpc(const NbiotProtocol::MqttMessage &msg) {
  const char *idStr = msg.topic + 26;  // lo que va detras de "v1/devices/me/rpc/request/"

  char method[24] = {0};
  NbiotProtocol::jsonString(msg.payload, "method", method, sizeof(method));

  char topic[NbiotProtocol::kMqttTopicLen];
  snprintf(topic, sizeof(topic), "%s%s", MQTT_TOPIC_DEVICE_RPC_RESP, idStr);

  char params[160] = {0};
  char reply[224];

  // ---- configurarModuloC: el protocolo A -> C acordado ----
  //
  // Modulo A se dirige a la PASARELA y nombra el nodo de destino dentro de params, en
  // vez de usar el topic de RPC de pasarela. La respuesta se manda EN EL ACTO -
  // "enviado" - y el resultado real llega despues como un registro commandLog.
  //
  // Esa respuesta inmediata es la idea del diseno, no un atajo. Un nodo duerme entre
  // ventanas de subida y puede estar a minutos de oir nada; un RPC abierto tanto
  // tiempo expira en ThingsBoard e informa de un fallo para una orden que en realidad
  // se entrego y se aplico.
  if (strcmp(method, "configurarModuloC") == 0) {
    handleConfigurarModuloC(msg, idStr);
    return;
  }

  if (strcmp(method, "cfg") != 0) {
    snprintf(reply, sizeof(reply),
             "{\"error\":\"unknown method '%s', expected 'configurarModuloC' or 'cfg'\"}", method);
    queueRpcReply(topic, reply);
    return;
  }
  if (!NbiotProtocol::jsonObject(msg.payload, "params", params, sizeof(params))) {
    snprintf(reply, sizeof(reply), "{\"error\":\"missing params\"}");
    queueRpcReply(topic, reply);
    return;
  }

  char applied[128] = {0};
  if (!applyOwnConfig(params, applied, sizeof(applied))) {
    snprintf(reply, sizeof(reply), "{\"error\":\"no recognized keys (want UPLINK and/or BATCH)\"}");
    queueRpcReply(topic, reply);
    return;
  }

  // Devolver los valores tras el recorte, para que un recorte sea visible.
  snprintf(reply, sizeof(reply), "{\"applied\":{%s}}", applied);
  queueRpcReply(topic, reply);
}

// Aplica {"UPLINK":300,"BATCH":6} a la configuracion en marcha de este modulo:
// UPLINK = segundos entre subidas, BATCH = lecturas por lote.
bool ModemNBIoTMqtt::applyOwnConfig(const char *params, char *applied, size_t cap) {
  const char *cur = params;
  char key[24], value[24];
  size_t pos = 0;
  bool any = false;

  auto appendKv = [&](const char *k, unsigned long v) {
    int n = snprintf(applied + pos, cap - pos, "%s\"%s\":%lu", pos ? "," : "", k, v);
    if (n > 0 && (size_t)n < cap - pos) pos += (size_t)n;
  };

  while (NbiotProtocol::jsonNextPair(cur, key, sizeof(key), value, sizeof(value))) {
    long v = strtol(value, nullptr, 10);
    if (strcmp(key, "UPLINK") == 0) {
      appendKv("UPLINK", setBatchSeconds((uint32_t)(v < 0 ? 0 : v)));
      any = true;
    } else if (strcmp(key, "BATCH") == 0) {
      appendKv("BATCH", setBatchReadings((uint16_t)(v < 0 ? 0 : v)));
      any = true;
    } else {
      Serial.printf("[nbiot-mqtt] rpc: unrecognized own-config key \"%s\", ignored\n", key);
    }
  }
  applied[pos] = '\0';
  return any;
}

// RPC dirigido a un dispositivo hijo = reenviarlo a ese nodo por LoRa.
//   {"device":"NodoC-1","data":{"id":42,"method":"cfg","params":{"INTERVAL":300}}}
// Se guarda en la cola; main.cpp lo entrega en la siguiente ventana del nodo.
void ModemNBIoTMqtt::handleGatewayRpc(const NbiotProtocol::MqttMessage &msg) {
  char device[24] = {0};
  char data[224] = {0};
  if (!NbiotProtocol::jsonString(msg.payload, "device", device, sizeof(device)) ||
      !NbiotProtocol::jsonObject(msg.payload, "data", data, sizeof(data))) {
    Serial.println("[nbiot-mqtt] gateway rpc missing device/data, ignored");
    return;
  }

  long rpcId = 0;
  NbiotProtocol::jsonInt(data, "id", rpcId);

  char reply[224];
  auto replyErr = [&](const char *err) {
    snprintf(reply, sizeof(reply), "{\"device\":\"%s\",\"id\":%ld,\"data\":{\"error\":\"%s\"}}", device, rpcId,
             err);
    queueRpcReply(MQTT_TOPIC_GATEWAY_RPC, reply);
  };

  char method[24] = {0};
  NbiotProtocol::jsonString(data, "method", method, sizeof(method));

  // Se aceptan LOS DOS nombres de metodo en LOS DOS topics.
  //
  // Por que topic llega una orden depende solo de a que dispositivo de ThingsBoard
  // se dirija Modulo A: la pasarela CON-1 llega a v1/devices/me/rpc/request/<id>, un
  // dispositivo hijo llega aqui por v1/gateway/rpc. El documento del protocolo
  // especifica configurarModuloC y describe el camino de la pasarela, pero un panel
  // construido por nodo se dirigira naturalmente al nodo - y rechazarlo por el nombre
  // seria negarse por un tecnicismo cuando la intencion no tiene dudas.
  //
  // Los dos llevan lo mismo: un nodo de destino y unas claves que aplicar.
  if (strcmp(method, "cfg") != 0 && strcmp(method, "configurarModuloC") != 0) {
    replyErr("unknown method, expected 'configurarModuloC' or 'cfg'");
    return;
  }

  char params[160] = {0};
  if (!NbiotProtocol::jsonObject(data, "params", params, sizeof(params))) {
    replyErr("missing params");
    return;
  }

  const int slot = freeNodeCmdSlot();
  if (slot < 0) {
    replyErr("command queue full");
    return;
  }

  // Traducir el objeto params a la cadena "CFG,K=V,..." que ya entiende
  // applyConfigCommand() de Modulo C. Las claves no se validan aqui: Modulo C ignora
  // lo que no reconoce y solo confirma lo que aplico, asi que el nodo sigue siendo
  // la autoridad sobre su propio vocabulario de configuracion.
  const char *cur = params;
  char key[24], value[24];
  size_t pos = 0;
  pos += (size_t)snprintf(_pendingCfgScratch, sizeof(_pendingCfgScratch), "CFG");
  bool any = false;
  while (NbiotProtocol::jsonNextPair(cur, key, sizeof(key), value, sizeof(value))) {
    int n = snprintf(_pendingCfgScratch + pos, sizeof(_pendingCfgScratch) - pos, ",%s=%s", key, value);
    if (n > 0 && (size_t)n < sizeof(_pendingCfgScratch) - pos) {
      pos += (size_t)n;
      any = true;
    }
  }
  if (!any) {
    _pendingCfgScratch[0] = '\0';
    replyErr("params object was empty");
    return;
  }

  NodeCommand &nc = _nodeCmds[slot];
  strncpy(nc.cfg, _pendingCfgScratch, sizeof(nc.cfg) - 1);
  nc.cfg[sizeof(nc.cfg) - 1] = '\0';
  strncpy(nc.device, device, sizeof(nc.device) - 1);
  nc.device[sizeof(nc.device) - 1] = '\0';
  nc.rpcId = rpcId;
  nc.queuedMs = millis();
  nc.awaitingAck = false;
  nc.viaCommandLog = false;
  nc.pending = true;

  Serial.printf("[nbiot-mqtt] queued for %s: \"%s\" (waiting for its next uplink window)\n", nc.device,
                nc.cfg);
}

// main.cpp acaba de mandar la orden de este nodo por LoRa: queda esperando su ACK.
void ModemNBIoTMqtt::onNodeCommandDelivered(const char *device) {
  const int i = deliverableIndexFor(device);
  if (i < 0) return;
  NodeCommand &nc = _nodeCmds[i];
  nc.awaitingAck = true;
  nc.lastSentMs = millis();
  if (nc.attempts < 255) nc.attempts++;
  if (nc.attempts == 1) {
    Serial.printf("[nbiot-mqtt] sent to %s over LoRa, awaiting ACK\n", nc.device);
  } else {
    Serial.printf("[nbiot-mqtt] RESENT to %s over LoRa (attempt %u - no ACK to the "
                  "previous one), awaiting ACK\n", nc.device, (unsigned)nc.attempts);
  }
}

// El nodo contesto NACK: entendio la orden pero no aplico nada. Se informa como
// "error" y se libera el hueco.
void ModemNBIoTMqtt::onNodeCommandNack(const char *device, const char *reason) {
  int i = awaitingAckIndexFor(device);
  if (i < 0) i = undeliveredIndexFor(device);
  if (i < 0) return;
  NodeCommand &nc = _nodeCmds[i];

  if (nc.viaCommandLog) {
    publishCommandLogApplied(nc.rpcId, "error", reason, nc.device);
  } else {
    char reply[224];
    snprintf(reply, sizeof(reply), "{\"device\":\"%s\",\"id\":%ld,\"data\":{\"error\":\"%s\"}}",
             nc.device, nc.rpcId, reason);
    queueRpcReply(MQTT_TOPIC_GATEWAY_RPC, reply);
  }
  Serial.printf("[nbiot-mqtt] %s REJECTED the command: %s\n", nc.device, reason);
  nc = NodeCommand{};
}

// El nodo contesto ACK: se informa de lo que aplico y se libera el hueco.
void ModemNBIoTMqtt::onNodeCommandAck(const char *device, const char *ackPayload) {
  // Emparejar el ACK con la orden que lo espera DE ESE NODO. Con varias ordenes en
  // curso a nodos distintos, liberar la equivocada daria por confirmado el requestId
  // equivocado: una orden pareceria haber funcionado mientras otra expiraba en
  // silencio.
  int i = awaitingAckIndexFor(device);
  if (i < 0) i = undeliveredIndexFor(device);  // el ACK llego antes de apuntar la entrega
  if (i < 0) return;
  NodeCommand &_nodeCmd = _nodeCmds[i];

  // "ACK,INTERVAL=300,BMV080=0" -> devolver los valores aplicados (tras el recorte)
  // como resultado del RPC, para que un recorte en el nodo se vea en ThingsBoard.
  const char *body = strncmp(ackPayload, "ACK,", 4) == 0 ? ackPayload + 4 : ackPayload;
  if (_nodeCmd.viaCommandLog) {
    // Lleva QUE se aplico, no solo que se aplico algo. El nodo solo confirma las
    // claves que de verdad tomo, asi que es la diferencia entre que Modulo A crea que
    // un umbral cambio y poder mostrar el valor que el nodo usa realmente.
    publishCommandLogApplied(_nodeCmd.rpcId, "confirmado", body, _nodeCmd.device);
  } else {
    char reply[224];
    snprintf(reply, sizeof(reply), "{\"device\":\"%s\",\"id\":%ld,\"data\":{\"applied\":\"%s\"}}",
             _nodeCmd.device, _nodeCmd.rpcId, body);
    queueRpcReply(MQTT_TOPIC_GATEWAY_RPC, reply);
  }

  Serial.printf("[nbiot-mqtt] %s acked: %s\n", _nodeCmd.device, body);
  _nodeCmd = NodeCommand{};
}

// Ordenes que llevan demasiado tiempo en cola (DOWNLINK_QUEUE_TIMEOUT_MS) sin
// confirmacion: se informan como timeout y se liberan.
void ModemNBIoTMqtt::tickNodeCommandTimeout() {
  // Cada hueco se cronometra por separado: un nodo inalcanzable no debe mantener viva
  // la orden de otro nodo mas alla de su propio plazo.
  for (uint8_t qi = 0; qi < kNodeCmdQueue; qi++) {
    NodeCommand &_nodeCmd = _nodeCmds[qi];
    if (!_nodeCmd.pending) continue;
    if (millis() - _nodeCmd.queuedMs < DOWNLINK_QUEUE_TIMEOUT_MS) continue;

  // Se construye una vez, antes de la bifurcacion, porque la linea de log del final
  // tambien lo usa - llegue la orden por el protocolo que llegue.
  char why[72];
  if (_nodeCmd.awaitingAck) {
    snprintf(why, sizeof(why), "delivered %u time(s), no ACK", (unsigned)_nodeCmd.attempts);
  } else {
    snprintf(why, sizeof(why), "node never uplinked - not delivered");
  }

  if (_nodeCmd.viaCommandLog) {
    // DECIR QUE MITAD FALLO. Antes se publicaba un "timeout" a secas, tirando el unico
    // dato que necesita el operador: si la orden llego a salir al aire. Modulo B ya lo
    // sabe - awaitingAck significa que el nodo hizo su subida, la orden salio en su
    // ventana de escucha y luego no dijo nada; sin el, el nodo no transmitio nunca.
    //
    // Piden respuestas opuestas. "Entregada, sin ACK" es un nodo vivo que rechaza o
    // se cuelga - mirar el nodo. "Nunca entregada" es un nodo que no transmite -
    // mirar la radio, la alimentacion o la ubicacion. Una sola palabra para las dos
    // te manda a la mitad equivocada, y esto es una torre en un bosque, no una placa
    // en una mesa.
    //
    // Va en `detail`, que el documento del protocolo deja libre, asi que los cuatro
    // valores de status documentados no cambian.
    publishCommandLogApplied(_nodeCmd.rpcId, "timeout", why, _nodeCmd.device);
  } else {
    char reply[224];
    snprintf(reply, sizeof(reply), "{\"device\":\"%s\",\"id\":%ld,\"data\":{\"error\":\"%s\"}}", _nodeCmd.device,
             _nodeCmd.rpcId,
             _nodeCmd.awaitingAck ? "delivered but the node never acked" : "node never uplinked - not delivered");
    queueRpcReply(MQTT_TOPIC_GATEWAY_RPC, reply);
  }
    Serial.printf("[nbiot-mqtt] command for %s timed out (%s)\n", _nodeCmd.device, why);
    _nodeCmd = NodeCommand{};
  }
}

static size_t appendI64(char *out, int64_t v);  // definida mas abajo, junto al constructor del JSON

// ---- Resultado de una orden (commandLog / seguimientoCmd) ----
//
// La mitad del resultado del protocolo acordado. El RPC en si se contesto "enviado"
// al llegar; esto es lo que dice de verdad si el nodo hizo lo que se le pidio.
//
// Se publica como TELEMETRIA, no como respuesta RPC, porque el RPC se cerro hace
// minutos - un nodo puede estar a toda una ventana de subida de oir una orden, y
// mantener un RPC abierto tanto tiempo es lo que hacia parecer fallos a ordenes que
// solo eran lentas.
//
// appendKvAsJson: convierte el cuerpo del ACK del nodo - "temp_pre_off=48,temp_pre_on=50"
// - en campos JSON:  ,"temp_pre_off":48,"temp_pre_on":50
//
// POR QUE COMO CLAVES SUELTAS Y NO SOLO EL TEXTO: Modulo A necesita saber el umbral
// que el nodo usa ahora, y `detail` solo lo lleva como texto que habria que
// interpretar. Como claves de telemetria cada uno es un valor que un panel puede
// enlazar, graficar y comparar con lo que pidio. Los nombres son el vocabulario del
// propio nodo - magnitud mas sufijo, los nombres de la hoja de umbrales - asi que aqui
// no se inventa nada.
//
// Los numeros salen sin comillas para que se guarden como numeros; lo que no se pueda
// leer limpiamente como numero va entre comillas en vez de emitir JSON roto. Estas
// parejas llegan por radio, asi que no se dan por bien formadas.
static size_t appendKvAsJson(char *out, size_t cap, const char *kv) {
  size_t pos = 0;
  const char *p = kv;
  while (*p && pos + 8 < cap) {
    const char *comma = strchr(p, ',');
    const char *end = comma ? comma : p + strlen(p);
    const char *eq = nullptr;
    for (const char *q = p; q < end; q++) {
      if (*q == '=') { eq = q; break; }
    }
    if (eq && eq > p && (end - eq) > 1) {
      const size_t klen = (size_t)(eq - p);
      const size_t vlen = (size_t)(end - eq - 1);

      // Es numerico? Se mira caracter a caracter en vez de fiarse del resto de strtod.
      bool numeric = vlen > 0;
      bool dot = false;
      for (size_t i = 0; i < vlen && numeric; i++) {
        const char c = eq[1 + i];
        if (c == '-' && i == 0) continue;
        if (c == '.' && !dot) { dot = true; continue; }
        if (c < '0' || c > '9') numeric = false;
      }

      const size_t need = klen + vlen + (numeric ? 4 : 6);
      if (pos + need >= cap) break;
      out[pos++] = ',';
      out[pos++] = '"';
      memcpy(out + pos, p, klen); pos += klen;
      out[pos++] = '"';
      out[pos++] = ':';
      if (!numeric) out[pos++] = '"';
      memcpy(out + pos, eq + 1, vlen); pos += vlen;
      if (!numeric) out[pos++] = '"';
      out[pos] = 0;
    }
    if (!comma) break;
    p = comma + 1;
  }
  return pos;
}

void ModemNBIoTMqtt::publishCommandLog(long requestId, const char *status) {
  publishCommandLogApplied(requestId, status, nullptr, nullptr);
}

// Publica el resultado de una orden en tres sitios: la telemetria de la pasarela, la
// telemetria del nodo y los atributos del nodo.
void ModemNBIoTMqtt::publishCommandLogApplied(long requestId, const char *status,
                                              const char *detail, const char *device) {
  // EL OBJETO SE CONSTRUYE UNA VEZ Y LOS TRES ENVIOS SE COMPONEN DE EL.
  //
  // Antes el espejo y los atributos reutilizaban `payload` entero recortandole
  // llaves. Mientras `payload` fue plano colo; en cuanto la v2 del protocolo le
  // metio su propio {"ts":...,"values":{...}} salieron las dos cosas rotas a la
  // vez, y asi se vieron en el log del 2026-10-07:
  //
  //   espejo:    {"NodoC-3":[{"ts":N,"values":{"ts":N,"values":{...}}}]}   <- duplicado
  //   atributos: {"NodoC-3":{"seguimientoCmd":{...}},"temp_pre_off":20,...}}  <- descuadrado
  //
  // Contar llaves a mano sobre una cadena que otro sitio construye es fragil por
  // naturaleza: el dia que cambia el formato, rompe en silencio y lejos de aqui.
  // Con una sola fuente no hay nada que recortar.
  //
  // appendI64() y no %lld: nano newlib no lo trae de forma fiable.

  // ---- el objeto, sin llaves exteriores: "seguimientoCmd":{...} ----
  char inner[224];
  size_t ip = (size_t)snprintf(inner, sizeof(inner),
                               "\"seguimientoCmd\":{\"requestId\":%ld", requestId);

  // moduloC_id es OBLIGATORIO en la v2: el requestId lo genera MODULO-A por cada
  // conexion, empieza en 0 y se reinicia al reconectar, asi que por si solo no
  // identifica un comando.
  if (device && *device && ip + 32 < sizeof(inner)) {
    ip += (size_t)snprintf(inner + ip, sizeof(inner) - ip, ",\"moduloC_id\":\"%s\"", device);
  }
  ip += (size_t)snprintf(inner + ip, sizeof(inner) - ip, ",\"status\":\"%s\"", status);

  // `detail` no esta en el documento: lleva las claves que el nodo aplico de
  // verdad, o el motivo del rechazo. El documento acota los valores de status, no
  // prohibe campos extra. Las comillas y barras se descartan porque un paquete
  // malformado no puede poder emitir telemetria invalida.
  if (detail && *detail && ip + 24 < sizeof(inner)) {
    ip += (size_t)snprintf(inner + ip, sizeof(inner) - ip, ",\"detail\":\"");
    for (const char *p = detail; *p && ip < sizeof(inner) - 8; p++) {
      if (*p == '"' || *p == '\\' || (unsigned char)*p < 0x20) continue;
      inner[ip++] = *p;
    }
    if (ip < sizeof(inner) - 4) inner[ip++] = '"';
  }
  if (ip < sizeof(inner) - 2) inner[ip++] = '}';
  inner[ip] = 0;

  // ---- los umbrales aplicados, como campos sueltos ----
  // Solo una confirmacion lleva valores aplicados: el detail de un timeout es
  // prosa y el de un error un motivo de rechazo, no umbrales.
  char extra[200];
  extra[0] = 0;
  if (detail && *detail && strcmp(status, "confirmado") == 0 && strchr(detail, '=')) {
    appendKvAsJson(extra, sizeof(extra), detail);
  }

  const bool stamped = _haveNetTime;
  const int64_t nowMs = stamped ? netNowMs() : 0;

  // ---- 1. telemetria del gateway: la forma exacta que exige la v2 ----
  // Sin reloj de red se manda plano y ThingsBoard sella al recibir, que es mejor
  // que emitir un ts de 1970: ordenaria el registro como el mas antiguo de todos.
  char payload[288];
  size_t pos = 0;
  if (stamped) {
    pos += (size_t)snprintf(payload, sizeof(payload), "{\"ts\":");
    pos += appendI64(payload + pos, nowMs);
    pos += (size_t)snprintf(payload + pos, sizeof(payload) - pos, ",\"values\":{%s}}", inner);
  } else {
    snprintf(payload, sizeof(payload), "{%s}", inner);
  }
  queueRpcReply(MQTT_TOPIC_DEVICE_TELEMETRY, payload);

  if (device && *device) {
    // ---- 2. el mismo registro en la telemetria del propio nodo ----
    // v1/devices/me/telemetry lleva el token del gateway, asi que seguimientoCmd
    // aterriza en el dispositivo GATEWAY y nunca en el nodo. Esto lo pone tambien
    // bajo el nodo, con los umbrales como claves propias para que se puedan
    // graficar.
    char mirror[560];
    if (stamped) {
      size_t mp = (size_t)snprintf(mirror, sizeof(mirror), "{\"%s\":[{\"ts\":", device);
      mp += appendI64(mirror + mp, nowMs);
      snprintf(mirror + mp, sizeof(mirror) - mp, ",\"values\":{%s%s}}]}", inner, extra);
    } else {
      snprintf(mirror, sizeof(mirror), "{\"%s\":[{%s%s}]}", device, inner, extra);
    }
    queueRpcReply("v1/gateway/telemetry", mirror);

    // ---- 3. atributos del nodo ----
    // La seccion 5 del documento dice que el dashboard lee ATRIBUTOS y que
    // MODULO-A los copia desde la telemetria. Escribirlos aqui quita esa
    // dependencia. Son atributos de CLIENTE: por MQTT un dispositivo no puede
    // escribir los de servidor.
    char attrs[560];
    snprintf(attrs, sizeof(attrs), "{\"%s\":{%s%s}}", device, inner, extra);
    queueRpcReply("v1/gateway/attributes", attrs);
  }

  Serial.printf("[nbiot-mqtt] seguimientoCmd %ld -> %s%s\n", requestId, status,
                (device && *device) ? " (gateway + nodo + atributos)" : "");
}

// ---- configurarModuloC: el protocolo A -> C ----
//   {"method":"configurarModuloC",
//    "params":{"moduloC_id":"NodoC-1","action":"frecuencia","value":"freq_alarma"}}
//
// Contesta el RPC en el acto con "enviado" y deja la orden en cola para la siguiente
// ventana de subida del nodo. El resultado llega despues como commandLog.
//
// Acciones: umbrales_de1variable (alias umbral), NxT_lectura, NxT_enviodatos,
// NxTglobalenviodatos (a todos los nodos), y frecuencia (antigua, aun aceptada).
void ModemNBIoTMqtt::handleConfigurarModuloC(const NbiotProtocol::MqttMessage &msg,
                                             const char *idStr) {
  const long requestId = strtol(idStr, nullptr, 10);

  char respTopic[NbiotProtocol::kMqttTopicLen];
  snprintf(respTopic, sizeof(respTopic), "%s%s", MQTT_TOPIC_DEVICE_RPC_RESP, idStr);

  char params[224] = {0};
  char device[24] = {0};
  char action[24] = {0};
  char value[32] = {0};
  char reply[224];

  auto refuse = [&](const char *err) {
    // Un rechazo se contesta en el propio RPC, no como commandLog: no se llego a
    // encolar nada, asi que no habra resultado que informar despues.
    snprintf(reply, sizeof(reply), "{\"status\":\"error\",\"error\":\"%s\"}", err);
    queueRpcReply(respTopic, reply);
    Serial.printf("[nbiot-mqtt] configurarModuloC %ld rechazado: %s\n", requestId, err);
  };

  if (!NbiotProtocol::jsonObject(msg.payload, "params", params, sizeof(params))) {
    refuse("missing params");
    return;
  }
  // moduloC_id NO es obligatorio para todas las acciones: NxTglobalenviodatos va
  // dirigido a todos los MODULO-C del MODULO-B y por eso no lo lleva. Se exige
  // mas abajo, una vez se sabe que accion es.
  NbiotProtocol::jsonString(params, "moduloC_id", device, sizeof(device));
  NbiotProtocol::jsonString(params, "action", action, sizeof(action));

  // jsonScalar y no jsonString: la v2 manda la N como NUMERO JSON (1.5), sin
  // comillas, y jsonString() exige comillas. Acepta las dos formas.
  NbiotProtocol::jsonScalar(params, "value", value, sizeof(value));

  const bool isGlobal = (strcmp(action, "NxTglobalenviodatos") == 0);
  if (!isGlobal && !device[0]) {
    refuse("missing moduloC_id");
    return;
  }

  // Una accion global necesita un hueco POR NODO, no uno.
  const uint8_t nodeCount = (uint8_t)(sizeof(NBIOT_NODE_NAMES) / sizeof(NBIOT_NODE_NAMES[0]));
  const uint8_t slotsNeeded = isGlobal ? nodeCount : 1;

  // La comprobacion de huecos libres NO puede ir aqui: necesita saber QUE ajuste
  // toca el comando, y eso sale del CFG, que todavia no se ha construido. Va justo
  // antes de encolar.

  // ---- accion -> el vocabulario CFG que ya entiende Modulo C ----
  char cfg[128] = {0};
  if (strcmp(action, "frecuencia") == 0) {
    // Los tres ritmos con nombre. ESTOS SEGUNDOS SON UNA ELECCION, NO DE LA
    // ESPECIFICACION, que nombra los ajustes sin dar valores: son la traduccion
    // razonable a CFG,INTERVAL, y lo evidente es revisarlos cuando se acuerden los
    // numeros reales. freq_alarma es el suelo LORA_TX_PERIOD_MIN_MS.
    // (OJO: CFG,INTERVAL ajusta un periodo heredado; en el nodo actual el envio lo
    // marca el calendario, ver LORA_TX_PERIOD_MS en el firmware de Modulo C.)
    unsigned secs = 0;
    if (strcmp(value, "freq_normal") == 0) secs = 300;
    else if (strcmp(value, "freq_prealarma") == 0) secs = 60;
    else if (strcmp(value, "freq_alarma") == 0) secs = 15;
    if (secs == 0) {
      refuse("value must be freq_normal, freq_prealarma or freq_alarma");
      return;
    }
    snprintf(cfg, sizeof(cfg), "CFG,INTERVAL=%u", secs);

  } else if (strcmp(action, "umbral") == 0 || strcmp(action, "umbrales_de1variable") == 0) {
    // LA MAGNITUD Y EL UMBRAL LLEGAN EN CAMPOS SEPARADOS, y hay que juntarlos antes
    // de que el nodo reconozca nada. Visto en vivo el 2026-10-05, y no es lo que
    // muestra el documento del protocolo:
    //
    //   {"action":"umbrales_de1variable","variable":"temp",
    //    "umbral_pre_off":48,"umbral_pre_on":50,
    //    "umbral_alarm_off":55,"umbral_alarm_on":60}
    //
    // El nodo nombra un umbral como <magnitud><sufijo> - temp_pre_on - asi que una
    // clave "umbral_pre_on" no coincide con ninguna regla y se ignora en silencio.
    // Todos los umbrales de la orden se perderian y el nodo los rechazaria con NACK,
    // lo que parece un fallo de firmware y es en realidad un desacuerdo de nombres.
    //
    // Asi que "umbral_" se reescribe como "<variable>_". Una clave que ya lleva su
    // magnitud (temp_pre_on, como en la propia tabla del panel) pasa sin tocar - las
    // dos formas funcionan.
    char variable[24] = {0};
    NbiotProtocol::jsonString(params, "variable", variable, sizeof(variable));

    const char *cur = params;
    char k[40], v[32];
    size_t pos = (size_t)snprintf(cfg, sizeof(cfg), "CFG");
    bool any = false;
    while (NbiotProtocol::jsonNextPair(cur, k, sizeof(k), v, sizeof(v))) {
      if (strcmp(k, "moduloC_id") == 0 || strcmp(k, "action") == 0 ||
          strcmp(k, "variable") == 0 || strcmp(k, "value") == 0) {
        continue;  // son de enrutado, no umbrales
      }

      int n;
      if (strncmp(k, "umbral_", 7) == 0 && variable[0]) {
        n = snprintf(cfg + pos, sizeof(cfg) - pos, ",%s_%s=%s", variable, k + 7, v);
      } else {
        n = snprintf(cfg + pos, sizeof(cfg) - pos, ",%s=%s", k, v);
      }
      if (n > 0 && (size_t)n < sizeof(cfg) - pos) {
        pos += (size_t)n;
        any = true;
      }
    }
    if (!any) {
      refuse("umbral with no threshold keys");
      return;
    }
    // (strncmp con longitud 0 siempre da 0: la condicion equivale a !variable[0].)
    if (strncmp(params, "", 0) == 0 && !variable[0]) {
      // No es fatal - las claves pueden llevar ya su magnitud - pero conviene
      // decirlo, porque si no la llevan el nodo las rechazara todas.
      Serial.println("[nbiot-mqtt] umbral without a \"variable\" field - keys must "
                     "already name their magnitude");
    }

  } else if (strcmp(action, "NxT_lectura") == 0) {
    // "Cada sensor se lee cada N x su periodo minimo de lectura. N >= 1."
    // La N viaja en crudo: multiplica el NODO, que es quien conoce sus periodos
    // base y quien puede validar y confirmar lo que de verdad aplico.
    if (!value[0]) {
      refuse("NxT_lectura sin value");
      return;
    }
    snprintf(cfg, sizeof(cfg), "CFG,NXT_LECTURA=%s", value);

  } else if (strcmp(action, "NxT_enviodatos") == 0 || isGlobal) {
    // Mismo comando para el nodo; lo unico que cambia es a cuantos va.
    // NxTglobalenviodatos "sustituye al antiguo action: frecuencia, que deja de
    // usarse" - de ahi que `frecuencia` siga aceptandose pero ya no se documente.
    if (!value[0]) {
      refuse(isGlobal ? "NxTglobalenviodatos sin value" : "NxT_enviodatos sin value");
      return;
    }
    snprintf(cfg, sizeof(cfg), "CFG,NXT_ENVIO=%s", value);

  } else {
    refuse("unknown action, expected 'umbrales_de1variable', 'NxT_lectura', "
           "'NxT_enviodatos' or 'NxTglobalenviodatos'");
    return;
  }

  // Ya se sabe que CFG se va a mandar, asi que ya se puede saber que ajuste toca
  // y cuantos huecos hacen falta de verdad.
  char supKey[24];
  supersedeKeyFromCfg(cfg, supKey, sizeof(supKey));

  // Los huecos que este comando puede REUTILIZAR por sustituir a otro del mismo
  // ajuste no cuentan como ocupados: si no, ordenes repetidas seguirian llenando
  // la cola aunque solo una vaya a sobrevivir.
  uint8_t reusables = 0;
  for (uint8_t t = 0; t < slotsNeeded; t++) {
    const char *target = isGlobal ? NBIOT_NODE_NAMES[t].name : device;
    if (supersedableIndexFor(target, supKey) >= 0) reusables++;
  }
  if (freeNodeCmdSlots() + reusables < slotsNeeded) {
    refuse("command queue full");
    return;
  }

  // UNA ENTRADA POR NODO DESTINO, TODAS CON EL MISMO requestId.
  //
  // El documento lo pide asi para la accion global: "Una sola respuesta
  // inmediata. Un seguimientoCmd por cada MODULO-C, cada uno en su propia
  // publicacion y todos con el mismo requestId." Como cada hueco reporta su
  // propio resultado, eso sale solo: basta con encolar uno por nodo.
  //
  // Y sirve para algo mas que la simetria - cada nodo puede responder distinto.
  // El ejemplo del propio documento lo ensena: NodoC-1 confirmado y NodoC-2
  // timeout, bajo el mismo numero de peticion.
  for (uint8_t t = 0; t < slotsNeeded; t++) {
    const char *target = isGlobal ? NBIOT_NODE_NAMES[t].name : device;

    // Sustituir antes que encolar. El comando anterior se cierra como `error`
    // diciendo por que: el documento acota status a confirmado/error/timeout, y
    // dejarlo sin respuesta seria peor - Modulo A lo mostraria "en transito"
    // para siempre, que es exactamente el sintoma que estamos arreglando.
    int slot = supersedableIndexFor(target, supKey);
    if (slot >= 0) {
      const long viejo = _nodeCmds[slot].rpcId;
      if (viejo != requestId) {
        char motivo[72];
        snprintf(motivo, sizeof(motivo), "sustituido por el comando %ld", requestId);
        // SIN `device`, a proposito: eso publica solo el registro del gateway en
        // vez de tres mensajes. Un comando sustituido no llego a aplicar nada, asi
        // que el espejo en el nodo y los atributos no tienen valores que llevar.
        //
        // Y evita desbordar el anillo de respuestas: una sustitucion global son 3
        // nodos, que a tres mensajes cada uno serian 9 mas el `enviado` = 10 en un
        // anillo de 8, y el mas viejo que se tira puede ser el `enviado` de otro
        // comando - justo el fallo del que avisa el comentario del anillo.
        publishCommandLogApplied(viejo, "error", motivo, nullptr);
        Serial.printf("[nbiot-mqtt] %ld sustituye al %ld en %s (misma accion)\n",
                      requestId, viejo, target);
      }
    } else {
      slot = freeNodeCmdSlot();
    }
    if (slot < 0) break;  // comprobado arriba; defensivo

    NodeCommand &nc = _nodeCmds[slot];
    strncpy(nc.device, target, sizeof(nc.device) - 1);
    nc.device[sizeof(nc.device) - 1] = 0;
    strncpy(nc.cfg, cfg, sizeof(nc.cfg) - 1);
    nc.cfg[sizeof(nc.cfg) - 1] = 0;
    strncpy(nc.supKey, supKey, sizeof(nc.supKey) - 1);
    nc.supKey[sizeof(nc.supKey) - 1] = 0;
    nc.rpcId = requestId;
    nc.queuedMs = millis();
    nc.awaitingAck = false;
    nc.attempts = 0;
    nc.lastSentMs = 0;
    nc.viaCommandLog = true;   // el resultado sale como seguimientoCmd, no por este RPC
    nc.pending = true;
  }

  // CONTESTAR YA. El nodo puede estar a minutos de su siguiente ventana de subida, y
  // un RPC abierto tanto tiempo expira en ThingsBoard e informa de un fallo para una
  // orden que se entrego y se aplico.
  snprintf(reply, sizeof(reply), "{\"status\":\"enviado\",\"requestId\":%ld}", requestId);
  queueRpcReply(respTopic, reply);

  if (isGlobal) {
    Serial.printf("[nbiot-mqtt] configurarModuloC %ld -> LOS %u NODOS: \"%s\" "
                  "(enviado; esperando sus ventanas)\n",
                  requestId, (unsigned)slotsNeeded, cfg);
  } else {
    Serial.printf("[nbiot-mqtt] configurarModuloC %ld -> %s: \"%s\" (enviado; esperando ventana)\n",
                  requestId, device, cfg);
  }
}

// Pone un mensaje en la cola de publicaciones (respuestas RPC y commandLog).
void ModemNBIoTMqtt::queueRpcReply(const char *topic, const char *payload) {
  if (_rpcReplyCount >= kRpcReplyQueue) {
    // Llena. Se va la MAS ANTIGUA, no la mas nueva: un registro nuevo sustituye a uno
    // viejo de la misma orden, y el mas nuevo es el que aun merece la pena mandar.
    Serial.println("[nbiot-mqtt] rpc reply queue full - dropping the oldest");
    popRpcReply();
  }
  const uint8_t slot = (uint8_t)((_rpcReplyHead + _rpcReplyCount) % kRpcReplyQueue);
  RpcReply &r = _rpcReplies[slot];
  strncpy(r.topic, topic, sizeof(r.topic) - 1);
  r.topic[sizeof(r.topic) - 1] = '\0';
  strncpy(r.payload, payload, sizeof(r.payload) - 1);
  r.payload[sizeof(r.payload) - 1] = '\0';
  _rpcReplyCount++;
}

// ---- Resultado del comando AT en curso ----

void ModemNBIoTMqtt::completeCommand(CmdOutcome outcome) {
  _cmd.active = false;
  _cmd.outcome = outcome;
}

void ModemNBIoTMqtt::checkCommandTimeout() {
  if (_cmd.active && millis() - _cmd.sentAtMs > _cmd.timeoutMs) completeCommand(CmdOutcome::TIMEOUT);
}

// Devuelve el resultado del ultimo comando una sola vez (y lo borra).
ModemNBIoTMqtt::CmdOutcome ModemNBIoTMqtt::consumeOutcome() {
  CmdOutcome o = _cmd.outcome;
  _cmd.outcome = CmdOutcome::NONE;
  return o;
}

// ---------- Maquina de estados ----------

void ModemNBIoTMqtt::setState(State s) {
  // Apuntar cada transicion. La OLED solo muestra el nombre del estado ACTUAL, asi que
  // un bucle rapido POWERING -> WAIT_AT -> ERROR -> OFF parece en pantalla un unico
  // estado atascado - que es exactamente como "NBIoT: WAIT_AT" se leyo como
  // "conectado a la banda NB-IoT". Por el puerto serie el ciclo no tiene duda.
  if (_state != s) {
    Serial.printf("[nbiot-mqtt] state %s -> %s\n", stateName(), stateNameOf(s));
    Serial.flush();
  }
  _state = s;
  _stateEnteredMs = millis();
  switch (s) {
    case State::ATTACHING:
      _cgpaddrChecked = false;
      _csqCheckedAfterAttach = false;
      _cclkChecked = false;
      _lastCeregPollMs = 0;
      break;
    case State::IDLE:
      _lastCsqPollMs = 0;  // una consulta poco despues de entrar en IDLE
      break;
    case State::PUBLISHING:
      _publishInFlight = false;
      break;
    case State::RPC_REPLY:
      _rpcReplyInFlight = false;
      break;
    default:
      break;
  }
}

// Dejar de conducir el UART del modem por completo. Cerrar el periferico no basta:
// el pin tambien tiene que dejar de ser salida, o conserva su ultimo nivel y sigue
// alimentando al modulo sin corriente.
void ModemNBIoTMqtt::releaseUart() {
  if (_uartStarted) {
    _serial.end();
    _uartStarted = false;
  }
  pinMode(_txPin, INPUT);
  pinMode(_rxPin, INPUT);
}

void ModemNBIoTMqtt::startUart() {
  if (_uartStarted) return;
  _serial.begin(_baud, SERIAL_8N1, _rxPin, _txPin);
  _uartStarted = true;
  while (_serial.available()) _serial.read();  // tirar el ruido del encendido
}

// Dar alimentacion al modem.
void ModemNBIoTMqtt::enterPowering() {
  // PRIMERO EL UART ABAJO, con la linea TX liberada, antes de tocar la alimentacion:
  // un pin TX conducido alimenta el modulo mientras VIN esta apagado e impide un
  // arranque en frio de verdad. El UART se vuelve a abrir en tickPowering() cuando el
  // rail ha subido y se ha asentado.
  releaseUart();

  digitalWrite(_channelPin, NBIOT_CHANNEL_ACTIVE);
  digitalWrite(_enPin, NBIOT_EN_ACTIVE);
  _sawBootUrc = false;
  _pwrkeyReleased = false;
  setState(State::POWERING);
}

void ModemNBIoTMqtt::enterError(const char *reason) {
  setLastError(reason);
  _consecutiveFailures++;
  _backoffUntilMs = millis() + computeBackoff(_consecutiveFailures);
  setState(State::ERROR);
}

void ModemNBIoTMqtt::powerDown() {
  // Soltar TX antes de cortar VIN: si no, el modulo se sigue alimentando por el y esto
  // no seria un apagado en absoluto.
  releaseUart();
  digitalWrite(_enPin, NBIOT_DISABLE);
  digitalWrite(_channelPin, !NBIOT_CHANNEL_ACTIVE);
  digitalWrite(_pwrkeyPin, !NBIOT_PWRKEY_ACTIVE);
  _attached = false;
  _mqttConnected = false;
  _powerCycleSettleUntilMs = millis() + NBIOT_POWER_OFF_SETTLE_MS;
}

// Ultimo recurso: apagar el modem y volver a empezar desde OFF.
void ModemNBIoTMqtt::powerCycle() {
  setLastError("power cycling modem (last resort)");
  powerDown();
  setState(State::OFF);
}

// Recuperacion escalonada: reintentar en el mismo nivel unas pocas veces y, si no,
// bajar un nivel (publicar -> reconectar MQTT -> registrarse -> apagar y encender).
void ModemNBIoTMqtt::handleFailureAtLevel(RecoveryLevel level, const char *reason) {
  setLastError(reason);
  _consecutiveFailures++;

  switch (level) {
    case RecoveryLevel::PUBLISH:
      _publishRetries++;
      if (_publishRetries <= NBIOT_MAX_SEND_RETRIES) {
        applyBackoffAndSetState(State::PUBLISHING);
        return;
      }
      _publishRetries = 0;
      _mqttConnected = false;  // forzar un QMTOPEN/QMTCONN nuevo, no solo otro intento de publicar
      _mqttConnectSub = MqttConnectSub::CLOSE_FIRST;
      applyBackoffAndSetState(State::MQTT_CONNECT);
      return;

    case RecoveryLevel::MQTT_CONNECT:
      _mqttConnectRetries++;
      if (_mqttConnectRetries <= NBIOT_MAX_SOCKET_RETRIES) {
        _mqttConnectSub = MqttConnectSub::CLOSE_FIRST;
        applyBackoffAndSetState(State::MQTT_CONNECT);
        return;
      }
      _mqttConnectRetries = 0;
      applyBackoffAndSetState(State::ATTACHING);
      return;

    case RecoveryLevel::ATTACH:
      _attachRetries++;
      if (_attachRetries <= NBIOT_MAX_ATTACH_RETRIES) {
        applyBackoffAndSetState(State::ATTACHING);
        return;
      }
      _attachRetries = 0;
      powerCycle();
      return;
  }
}

void ModemNBIoTMqtt::applyBackoffAndSetState(State s) {
  _backoffUntilMs = millis() + computeBackoff(_consecutiveFailures);
  setState(s);
}

// Espera creciente: base x 2^intento, con un techo.
uint32_t ModemNBIoTMqtt::computeBackoff(uint32_t attempt) const {
  if (attempt > 10) attempt = 10;
  uint32_t v = (uint32_t)NBIOT_BACKOFF_BASE_MS << attempt;
  if (v > NBIOT_BACKOFF_MAX_MS || v < NBIOT_BACKOFF_BASE_MS) v = NBIOT_BACKOFF_MAX_MS;
  return v;
}

void ModemNBIoTMqtt::setLastError(const char *msg) {
  strncpy(_lastError, msg, sizeof(_lastError) - 1);
  _lastError[sizeof(_lastError) - 1] = '\0';
  // Tambien IMPRIMIRLO. Antes solo se guardaba el texto, y por eso un modem que no
  // contestaba nunca producia un log con nada mas que "-> AT": el abandono, el
  // apagado y el motivo eran invisibles, y el unico sitio donde aparecia el error era
  // una funcion de consulta que nadie llamaba. Un bucle de recuperacion silencioso no
  // se distingue de uno colgado.
  Serial.printf("[nbiot-mqtt] ERROR: %s\n", msg);
  Serial.flush();
}

const char *ModemNBIoTMqtt::stateName() const { return stateNameOf(_state); }

const char *ModemNBIoTMqtt::stateNameOf(State st) {
  switch (st) {
    case State::OFF: return "OFF";
    case State::POWERING: return "POWERING";
    case State::WAIT_AT: return "WAIT_AT";
    case State::CONFIG: return "CONFIG";
    case State::ATTACHING: return "ATTACHING";
    case State::MQTT_CONNECT: return "MQTT_CONNECT";
    case State::IDLE: return "IDLE";
    case State::PUBLISHING: return "PUBLISHING";
    case State::RPC_REPLY: return "RPC_REPLY";
    case State::ERROR: return "ERROR";
  }
  return "?";
}

uint32_t ModemNBIoTMqtt::attachedUptimeMs() const {
  return _attached ? (millis() - _attachedSinceMs) : 0;
}

// ---- OFF: esperar a que se descargue tras un apagado y volver a encender ----
void ModemNBIoTMqtt::tickOff() {
  if (millis() < _powerCycleSettleUntilMs) return;
  enterPowering();
}

// ---- POWERING: pulso de PWRKEY y espera al arranque ----
void ModemNBIoTMqtt::tickPowering() {
  // Pulso de PWRKEY sin bloquear: esperar a que VIN y el canal se asienten, poner
  // PWRKEY a bajo NBIOT_PWRKEY_PULSE_MS, soltarlo, y esperar un aviso de arranque o un
  // plazo fijo antes de seguir. Los reintentos de AT de WAIT_AT son el verdadero
  // punto de sincronizacion; esto es solo una ventaja de salida.
  // VIN y la puerta del canal tienen que estar estables ANTES de pulsar PWRKEY: el
  // modulo puede ignorar del todo un pulso sobre un rail que aun esta subiendo.
  // Subido de 300 a 2000 (2026-09-11) junto con el techo de WAIT_AT; los dos formaban
  // parte de la misma carrera de encendido demasiado agresiva.
  // (Nota: segun Config.h, PWRKEY no esta cableado en esta placa; el pulso es
  // inofensivo.)
  static const uint32_t kPowerSettleMs = 2000;
  uint32_t elapsed = millis() - _stateEnteredMs;

  if (elapsed < kPowerSettleMs) return;

  if (elapsed < kPowerSettleMs + NBIOT_PWRKEY_PULSE_MS) {
    digitalWrite(_pwrkeyPin, NBIOT_PWRKEY_ACTIVE);
    return;
  }

  if (!_pwrkeyReleased) {
    digitalWrite(_pwrkeyPin, !NBIOT_PWRKEY_ACTIVE);
    _pwrkeyReleased = true;
  }

  if (_sawBootUrc || elapsed > NBIOT_TIMEOUT_POWERING_MS) {
    // Solo ahora, con el modulo alimentado y asentado, es seguro conducir TX.
    startUart();
    _lastAtPingMs = 0;
    setState(State::WAIT_AT);
  }
}

// ---- WAIT_AT: preguntar "AT" hasta que conteste ----
void ModemNBIoTMqtt::tickWaitAt() {
  if (millis() - _stateEnteredMs > NBIOT_TIMEOUT_WAIT_AT_MS) {
    enterError("modem never responded to AT");
    return;
  }
  if (_cmd.active) return;

  if (consumeOutcome() == CmdOutcome::OK) {
    _configStep = ConfigStep::ATE0;
    _configStepEnteredMs = millis();
    setState(State::CONFIG);
    return;
  }

  if (millis() - _lastAtPingMs >= NBIOT_AT_PING_GAP_MS) {
    issueCommand("AT", NBIOT_AT_CMD_TIMEOUT_MS);
    _lastAtPingMs = millis();
  }
}

// ---- CONFIG: la secuencia de configuracion, un comando por paso ----
//   ATE0 -> CMEE=2 -> CTZU=1 -> QSCLK=0 -> CFUN=1 -> CPIN? -> CFUN=0 ->
//   QCGDEFCONT (APN) -> CFUN=1 -> ATTACHING
void ModemNBIoTMqtt::tickConfig() {
  if (millis() - _stateEnteredMs > NBIOT_TIMEOUT_CONFIG_MS) {
    enterError("CONFIG timed out");
    return;
  }
  if (_cmd.active) return;

  if (_cmd.outcome != CmdOutcome::NONE) {
    CmdOutcome outcome = consumeOutcome();
    // CTZU y QSCLK son "si se puede" (ver sus casos abajo); todos los demas pasos
    // son obligatorios, y un ERROR o un plazo agotado en ellos es fatal.
    if (outcome != CmdOutcome::OK && _configStep != ConfigStep::QSCLK &&
        _configStep != ConfigStep::CTZU) {
      enterError("a CONFIG step returned ERROR/timeout");
      return;
    }
    _configStep = (ConfigStep)((uint8_t)_configStep + 1);
    _configStepEnteredMs = millis();
  }

  switch (_configStep) {
    case ConfigStep::ATE0:
      issueCommand("ATE0", NBIOT_AT_CMD_TIMEOUT_MS);
      break;
    case ConfigStep::CMEE:
      issueCommand("AT+CMEE=2", NBIOT_AT_CMD_TIMEOUT_MS);  // errores con texto ("ue not power on"...), no un ERROR a secas
      break;
    case ConfigStep::CTZU:
      // Actualizacion automatica de zona y hora desde la red, para que AT+CCLK? de
      // despues devuelva la hora real con la que fechar cada lectura. Antes de CFUN=1
      // para que se sincronice durante el registro. Si se puede: si el modulo lo
      // rechaza, simplemente no se pone "ts" por lectura (ver buildGatewayPayload).
      issueCommand("AT+CTZU=1", NBIOT_AT_CMD_TIMEOUT_MS);
      break;
    case ConfigStep::QSCLK:
      // AJUSTE DE BANCO, NO UNA DECISION DE DISENO: la pasarela funciona con energia
      // solar y un modem que no duerme nunca es un consumo real y continuo. Esto
      // desactiva el sueno para siempre solo porque obtener una puesta en marcha
      // limpia del camino de QMTPUB importaba mas que el consumo. El diseno real es:
      // dejar el sueno activo, despertar el modem explicitamente antes de publicar,
      // confirmar que esta despierto y publicar - eso es otra iteracion, no esta hecha.
      //
      // Sin esto el modem puede entrar solo en sueno profundo en mitad de un comando
      // (confirmado el 2026-08-19: un AT+QMTPUB mandado mientras dormia no recibio
      // nunca el '>' - solo un "+QNBIOTEVENT: EXIT DEEPSLEEP" cuando desperto por su
      // cuenta, con la publicacion perdida y sin error ni timeout). Si no se admite,
      // se sigue igualmente en vez de bloquear la puesta en marcha.
      issueCommand("AT+QSCLK=0", NBIOT_AT_CMD_TIMEOUT_MS);
      break;
    case ConfigStep::CFUN_ON_1:
      // TIENE que ir antes de AT+CPIN?: este modulo no activa solo su radio al
      // arrancar (confirmado el 2026-08-19: AT+CPIN? falla con
      // "+CME ERROR: ue not power on" sin esto antes).
      issueCommand("AT+CFUN=1", NBIOT_AT_CMD_TIMEOUT_MS);
      break;
    case ConfigStep::CPIN:
      issueCommand("AT+CPIN?", NBIOT_AT_CMD_TIMEOUT_MS);  // SIM lista?
      break;
    case ConfigStep::CFUN_OFF:
      issueCommand("AT+CFUN=0", NBIOT_AT_CMD_TIMEOUT_MS);  // el APN solo se acepta con la radio apagada
      break;
    case ConfigStep::QCGDEFCONT: {
      // Dejar que se asiente la transicion de la SIM tras CFUN=0 (dice NOT READY un
      // momento): confirmado el 2026-08-19 que necesita ~3 s, o AT+QCGDEFCONT llega
      // antes de tiempo. Sin bloquear: se vuelve a mirar hasta que toca.
      if (millis() - _configStepEnteredMs < 3000) return;
      char cmd[64];
      snprintf(cmd, sizeof(cmd), "AT+QCGDEFCONT=\"IP\",\"%s\"", NBIOT_APN);
      issueCommand(cmd, NBIOT_AT_CMD_TIMEOUT_MS);
      break;
    }
    case ConfigStep::CFUN_ON_2:
      issueCommand("AT+CFUN=1", NBIOT_AT_CMD_TIMEOUT_MS);
      break;
    case ConfigStep::DONE:
      setState(State::ATTACHING);
      break;
  }
}

// ---- ATTACHING: esperar al registro en la red, y leer cobertura y hora ----
//   AT+CEREG? (hasta registrado) -> AT+CGPADDR -> AT+CSQ -> AT+CCLK? -> MQTT_CONNECT
void ModemNBIoTMqtt::tickAttaching() {
  if (millis() < _backoffUntilMs) return;

  if (!_cgpaddrChecked) {
    if (millis() - _stateEnteredMs > NBIOT_TIMEOUT_ATTACH_MS) {
      handleFailureAtLevel(RecoveryLevel::ATTACH, "attach timed out");
      return;
    }
    if (_cmd.active) return;

    if (_cmd.outcome == CmdOutcome::NONE) {
      if (millis() - _lastCeregPollMs >= NBIOT_ATTACH_POLL_INTERVAL_MS) {
        issueCommand("AT+CEREG?", NBIOT_AT_CMD_TIMEOUT_MS);
        _lastCeregPollMs = millis();
      }
      return;
    }

    if (consumeOutcome() != CmdOutcome::OK) {
      handleFailureAtLevel(RecoveryLevel::ATTACH, "AT+CEREG? failed");
      return;
    }

    NbiotProtocol::CeregState st;
    if (!NbiotProtocol::parseCereg(_cmd.infoLine, st) || !NbiotProtocol::cieregIsAttached(st)) {
      _attached = false;
      return;  // aun no registrado: seguir preguntando; el plazo de arriba es la red de seguridad
    }

    _attached = true;
    _attachedSinceMs = millis();
    _attachRetries = 0;
    _consecutiveFailures = 0;

    char cmd[24];
    snprintf(cmd, sizeof(cmd), "AT+CGPADDR=%d", NBIOT_CONTEXT_ID);
    issueCommand(cmd, NBIOT_AT_CMD_TIMEOUT_MS);
    _cgpaddrChecked = true;
    return;
  }

  if (_cmd.active) return;
  if (_cmd.outcome == CmdOutcome::NONE) return;

  if (!_csqCheckedAfterAttach) {
    consumeOutcome();  // diga lo que diga CGPADDR, CEREG ya confirmo el registro: no merece fallar por ello
    // Un AT+CSQ justo aqui, sin esperar a que MQTT_CONNECT funcione: si no, la
    // lectura de cobertura (y lo que la muestra, como las barras de la OLED) no se
    // actualizaria nunca mientras Modulo A o el servidor fueran inalcanzables, aunque
    // el modem este registrado con senal real.
    issueCommand("AT+CSQ", NBIOT_AT_CMD_TIMEOUT_MS);
    _csqCheckedAfterAttach = true;
    return;
  }

  if (!_cclkChecked) {
    CmdOutcome csqOutcome = consumeOutcome();
    if (csqOutcome == CmdOutcome::OK && _cmd.hasInfoLine) {
      int dbm;
      if (NbiotProtocol::parseCsq(_cmd.infoLine, dbm)) _rssiDbm = dbm;
    }
    // Un AT+CCLK? por registro, para fijar la relacion epoca<->millis() con la que se
    // fecha cada lectura en la subida MQTT. Si se puede: un fallo (o un reloj
    // increible) deja _haveNetTime a false y la subida va sin "ts".
    issueCommand("AT+CCLK?", NBIOT_AT_CMD_TIMEOUT_MS);
    _cclkChecked = true;
    return;
  }

  CmdOutcome cclkOutcome = consumeOutcome();
  if (cclkOutcome == CmdOutcome::OK && _cmd.hasInfoLine) {
    int64_t epochMs;
    if (NbiotProtocol::parseCclk(_cmd.infoLine, epochMs) && epochMs >= NBIOT_MIN_VALID_EPOCH_MS) {
      _netEpochMsAtSync = epochMs;
      _netSyncLocalMs = millis();
      _haveNetTime = true;
      Serial.printf("[nbiot-mqtt] network time acquired: epoch %lld ms - uplink readings will carry per-reading ts\n",
                    (long long)epochMs);
    } else {
      Serial.println("[nbiot-mqtt] no usable network time (AT+CCLK?) - uplink will use ThingsBoard receipt time");
    }
  }

  _mqttConnectSub = MqttConnectSub::KEEPALIVE_CFG;
  setState(State::MQTT_CONNECT);
}

// "Ahora" en ms de epoca: la hora de red de la ultima sincronizacion mas el millis()
// transcurrido desde entonces.
int64_t ModemNBIoTMqtt::netNowMs() const {
  return _netEpochMsAtSync + (int64_t)(int32_t)(millis() - _netSyncLocalMs);
}

// ---- MQTT_CONNECT: abrir la sesion con ThingsBoard, paso a paso ----
//   QMTCLOSE -> keepalive -> sesion persistente -> QMTOPEN -> QMTCONN ->
//   suscripciones (RPC propio y RPC de pasarela) -> anunciar cada nodo -> IDLE
void ModemNBIoTMqtt::tickMqttConnect() {
  if (millis() < _backoffUntilMs) return;
  if (millis() - _stateEnteredMs > NBIOT_TIMEOUT_SOCKET_MS) {
    handleFailureAtLevel(RecoveryLevel::MQTT_CONNECT, "MQTT connect timed out");
    return;
  }
  if (_cmd.active) return;

  switch (_mqttConnectSub) {
    case MqttConnectSub::CLOSE_FIRST: {
      // Cierre defensivo antes de cada intento de (re)conexion: el estado del cliente
      // MQTT vive en el modem, no en el ESP32, y sobrevive a un reflasheo o reinicio
      // del ESP32. Una sesion sobrante de un intento anterior dejaba bloqueado el
      // client_idx 0, y AT+QMTOPEN fallaba sin mas con
      // "+CME ERROR: operation not allowed" hasta que se anadio esto (2026-08-19). Si
      // se puede: un ERROR aqui solo significa que no habia nada que cerrar.
      if (_cmd.outcome != CmdOutcome::NONE) {
        consumeOutcome();
        _mqttConnectSub = MqttConnectSub::KEEPALIVE_CFG;
        return;
      }
      char cmd[24];
      snprintf(cmd, sizeof(cmd), "AT+QMTCLOSE=%d", MQTT_CLIENT_IDX);
      issueCommand(cmd, NBIOT_AT_CMD_TIMEOUT_MS);
      return;
    }

    case MqttConnectSub::KEEPALIVE_CFG: {
      if (_cmd.outcome != CmdOutcome::NONE) {
        consumeOutcome();  // si se puede: seguir aunque se rechace el QMTCFG
        _mqttConnectSub = MqttConnectSub::SESSION_CFG;
        return;
      }
      char cmd[48];
      snprintf(cmd, sizeof(cmd), "AT+QMTCFG=\"keepalive\",%d,%lu", MQTT_CLIENT_IDX, (unsigned long)MQTT_KEEPALIVE_S);
      issueCommand(cmd, NBIOT_AT_CMD_TIMEOUT_MS);
      return;
    }

        case MqttConnectSub::SESSION_CFG: {
      // SESION PERSISTENTE - setCleanSession(false) en el documento del protocolo.
      //
      // clean_session = 0 le dice al servidor que CONSERVE la sesion de este cliente
      // a traves de una desconexion: sus suscripciones sobreviven, y los mensajes QoS 1
      // publicados mientras esta fuera un momento se ponen EN COLA en vez de tirarse.
      //
      // Aqui importa mas que en un enlace con cable. Este modulo no manda PINGREQ - ver
      // MQTT_KEEPALIVE_S - asi que la conexion solo la mantiene viva su propia
      // telemetria, y un enlace NB-IoT que se cae entre publicaciones no se nota hasta
      // el siguiente envio. Con sesion limpia, el servidor tiraria todas las ordenes
      // mandadas en ese hueco sin avisar a nadie. Con sesion persistente, estan
      // esperando cuando vuelve el enlace.
      //
      // Si se puede: un modem que rechace el ajuste sigue teniendo una subida que
      // funciona, y negarse a conectar por ello cambiaria una bajada fragil por no
      // tener enlace en absoluto.
      if (_cmd.outcome != CmdOutcome::NONE) {
        if (consumeOutcome() != CmdOutcome::OK) {
          Serial.println("[nbiot-mqtt] persistent session REJECTED - commands sent while "
                         "briefly offline will be lost");
        }
        _mqttConnectSub = MqttConnectSub::OPEN;
        return;
      }
      char cmd[48];
      snprintf(cmd, sizeof(cmd), "AT+QMTCFG=\"session\",%d,0", MQTT_CLIENT_IDX);
      issueCommand(cmd, NBIOT_AT_CMD_TIMEOUT_MS);
      return;
    }

    case MqttConnectSub::OPEN: {
      if (_cmd.outcome != CmdOutcome::NONE) {
        if (consumeOutcome() != CmdOutcome::OK) {
          handleFailureAtLevel(RecoveryLevel::MQTT_CONNECT, "QMTOPEN failed");
          return;
        }
        // CRITICO: encadenar directamente con QMTCONN, sin pausa ni espera - el
        // servidor cierra el socket ~5-6 s despues de QMTOPEN si no llega el CONNECT
        // (+QMTSTAT: <idx>,7). La unica demora es la del siguiente tick(), que en la
        // practica es inmediata.
        _mqttConnectSub = MqttConnectSub::CONN;
        return;
      }
      char cmd[96];
      snprintf(cmd, sizeof(cmd), "AT+QMTOPEN=%d,\"%s\",%d", MQTT_CLIENT_IDX, MQTT_BROKER_HOST, MQTT_BROKER_PORT);
      issueCommand(cmd, NBIOT_TIMEOUT_SOCKET_MS, CmdKind::QMTOPEN);
      return;
    }

    case MqttConnectSub::CONN: {
      if (_cmd.outcome != CmdOutcome::NONE) {
        if (consumeOutcome() != CmdOutcome::OK) {
          handleFailureAtLevel(RecoveryLevel::MQTT_CONNECT, "QMTCONN rejected/failed");
          return;
        }
        _mqttConnected = true;
        _mqttConnectRetries = 0;
        _consecutiveFailures = 0;
        _mqttConnectSub = MqttConnectSub::SUB_DEVICE;  // activar la bajada de ordenes antes de IDLE
        return;
      }
      // Autenticacion de ThingsBoard por token de acceso: el token va como usuario,
      // sin contrasena. (2026-08-19: se probo a poner una contrasena "" explicita tras
      // un CONNACK 5 "no autorizado" con un token correcto, y el modem lo rechazo con
      // "+CME ERROR: Incorrect parameters", asi que esta forma de 3 argumentos es la
      // correcta. La causa real del CONNACK 5 fue muy probablemente una sesion vieja
      // abierta en el modem desde un flasheo anterior - ver CLOSE_FIRST arriba.)
      // OJO: issueCommand() imprime el comando por el puerto serie, asi que el token
      // aparece en el log de depuracion.
      char cmd[128];
      snprintf(cmd, sizeof(cmd), "AT+QMTCONN=%d,\"%s\",\"%s\"", MQTT_CLIENT_IDX, MQTT_CLIENT_ID, MQTT_ACCESS_TOKEN);
      issueCommand(cmd, NBIOT_TIMEOUT_SOCKET_MS, CmdKind::QMTCONN);
      return;
    }

    // Las dos suscripciones son "si se puede": perder la bajada cuesta el control
    // remoto, pero la telemetria es el trabajo real de este modulo y no debe
    // bloquearla un SUBSCRIBE rechazado. Los fallos se apuntan y se sigue hacia IDLE.
    case MqttConnectSub::SUB_DEVICE: {
      if (_cmd.outcome != CmdOutcome::NONE) {
        if (consumeOutcome() != CmdOutcome::OK)
          Serial.println("[nbiot-mqtt] subscribe to this device's RPC topic FAILED - no remote config of Module B");
        _mqttConnectSub = MqttConnectSub::SUB_GATEWAY;
        return;
      }
      char cmd[96];
      snprintf(cmd, sizeof(cmd), "AT+QMTSUB=%d,%u,\"%s\",%d", MQTT_CLIENT_IDX, _subMsgId++,
               MQTT_TOPIC_DEVICE_RPC_SUB, MQTT_DOWNLINK_QOS);
      issueCommand(cmd, NBIOT_TIMEOUT_SOCKET_MS, CmdKind::QMTSUB);
      return;
    }

    case MqttConnectSub::SUB_GATEWAY: {
      if (_cmd.outcome != CmdOutcome::NONE) {
        if (consumeOutcome() != CmdOutcome::OK)
          Serial.println("[nbiot-mqtt] subscribe to the gateway RPC topic FAILED - no remote config of the nodes");
        else
          Serial.println("[nbiot-mqtt] downlink live - announcing the nodes next");
        _announceIdx = 0;
        _mqttConnectSub = MqttConnectSub::ANNOUNCE;
        return;
      }
      char cmd[96];
      snprintf(cmd, sizeof(cmd), "AT+QMTSUB=%d,%u,\"%s\",%d", MQTT_CLIENT_IDX, _subMsgId++,
               MQTT_TOPIC_GATEWAY_RPC, MQTT_DOWNLINK_QOS);
      issueCommand(cmd, NBIOT_TIMEOUT_SOCKET_MS, CmdKind::QMTSUB);
      return;
    }

    case MqttConnectSub::ANNOUNCE: {
      // Un v1/gateway/connect por nodo, uno por cada pasada por este estado.
      //
      // SIN ESTO, UN RPC A UN NODO NO LLEGA NUNCA. ThingsBoard solo encamina una orden
      // a un dispositivo hijo de una pasarela cuando la pasarela ha dicho que ese
      // dispositivo esta conectado a traves de ella. La telemetria no necesita ese
      // anuncio, asi que los nodos aparecian con datos y sanos mientras todas las
      // ordenes a ellos se tiraban en el servidor - y durante dos sesiones parecio un
      // fallo de firmware en este extremo.
      //
      // Se repite en cada conexion MQTT, porque el anuncio va ligado a la sesion: si
      // no, una reconexion des-registra en silencio todos los nodos.
      if (_cmd.outcome != CmdOutcome::NONE) {
        consumeOutcome();  // un anuncio fallido no es fatal: la telemetria sigue fluyendo
        _announceIdx++;
      }

      const size_t nodeCount = sizeof(NBIOT_NODE_NAMES) / sizeof(NBIOT_NODE_NAMES[0]);
      if (_announceIdx >= nodeCount) {
        Serial.printf("[nbiot-mqtt] %u node(s) announced - downlink live\n", (unsigned)nodeCount);
        setState(State::IDLE);
        return;
      }

      const char *name = NBIOT_NODE_NAMES[_announceIdx].name;
      _publishPayloadLen = (size_t)snprintf((char *)_publishPayload, sizeof(_publishPayload),
                                            "{\"device\":\"%s\"}", name);
      char header[96];
      snprintf(header, sizeof(header), "AT+QMTPUB=%d,0,0,0,\"v1/gateway/connect\"", MQTT_CLIENT_IDX);
      Serial.printf("[nbiot-mqtt] announcing %s to the gateway\n", name);
      issueCommand(header, NBIOT_TIMEOUT_SEND_MS, CmdKind::QMTPUB, _publishPayload, _publishPayloadLen);
      return;
    }
  }
}

// ---- IDLE: esperar; reconectar si la sesion cayo, contestar, o publicar si toca ----
void ModemNBIoTMqtt::tickIdle() {
  if (_cmd.active) return;

  CmdOutcome outcome = consumeOutcome();
  if (outcome == CmdOutcome::OK && _cmd.hasInfoLine) {
    int dbm;
    if (NbiotProtocol::parseCsq(_cmd.infoLine, dbm)) _rssiDbm = dbm;
  }

  // UNA SESION MUERTA SE RECONSTRUYE AQUI, no se deja para la siguiente publicacion.
  //
  // handleUrc() pone _mqttConnected a false cuando llega +QMTSTAT y dice que se
  // reconectara "en la siguiente publicacion". Pero solo se publica cuando llegan
  // datos por LoRa, e IDLE es justo el estado en el que no llegan. Un nodo que pasa
  // horas sin lecturas - o que simplemente esta apagado - significa que no se publica,
  // y por tanto que no se reconecta NUNCA: la maquina de estados se queda aqui
  // preguntando CSQ con una sesion muerta y la OLED diciendo "MQTT: caido" para siempre.
  //
  // Visto el 2026-09-14: Modulo B en IDLE durante dias, la red movil bien (CSQ 31/31,
  // registrado), MQTT caido en pantalla y sin reintentarse nunca, porque nada
  // transmitia para provocar una publicacion.
  //
  // Va condicionado a la espera creciente compartida, para que un servidor caido de
  // verdad se reintente con calma en vez de machacarse: los fallos de MQTT_CONNECT
  // alimentan esa misma espera a traves de handleFailureAtLevel().
  if (!_mqttConnected && millis() >= _backoffUntilMs) {
    Serial.println("[nbiot-mqtt] idle with a dead MQTT session - reconnecting now "
                   "(not waiting for a publish)");
    Serial.flush();
    _mqttConnectSub = MqttConnectSub::CLOSE_FIRST;
    setState(State::MQTT_CONNECT);
    return;
  }

  // Una respuesta RPC sale antes que el lote de telemetria: es lo que alguien esta
  // esperando en la pantalla de ThingsBoard, y el lote no pierde nada por esperar una
  // vuelta mas.
  if (rpcReplyPending() && _mqttConnected) {
    setState(State::RPC_REPLY);
    return;
  }

  // _flushRequested se salta la espera del lote - ver requestFlush(). Se borra cuando
  // la publicacion sale bien, no aqui, para que una publicacion urgente que falla se
  // reintente en vez de olvidarse sin decir nada.
  bool batchDue = _ringCount > 0 && (_flushRequested ||
                                      _ringCount >= _batchReadingsTarget ||
                                      (millis() - _oldestPendingMs) >= _batchSecondsTarget * 1000UL);
  if (batchDue) {
    setState(State::PUBLISHING);
    return;
  }

  // Mientras tanto, refrescar la cobertura para el panel cada cierto tiempo.
  if (millis() - _lastCsqPollMs >= NBIOT_CSQ_POLL_INTERVAL_MS) {
    issueCommand("AT+CSQ", NBIOT_AT_CMD_TIMEOUT_MS);
    _lastCsqPollMs = millis();
  }
}

// ---- PUBLISHING: subir el anillo a v1/gateway/telemetry, por trozos ----
void ModemNBIoTMqtt::tickPublishing() {
  if (millis() < _backoffUntilMs) return;

  if (!_mqttConnected) {
    // Reconexion perezosa: no intentar nunca publicar sobre una sesion que ya se sabe
    // muerta. La comprobacion batchDue de IDLE vuelve aqui en cuanto MQTT_CONNECT
    // funciona.
    _mqttReconnects++;
    _mqttConnectSub = MqttConnectSub::KEEPALIVE_CFG;
    setState(State::MQTT_CONNECT);
    return;
  }

  if (_cmd.active) return;

  if (!_publishInFlight) {
    _publishPayloadLen = buildGatewayPayload(_publishPayload, sizeof(_publishPayload), MQTT_PUB_CHUNK_READINGS);
    if (_publishPayloadLen == 0) {
      setState(State::IDLE);  // nada que publicar - no deberia pasar, batchDue implica anillo no vacio
      return;
    }
    char header[48];
    snprintf(header, sizeof(header), "AT+QMTPUB=%d,0,0,0,\"v1/gateway/telemetry\"", MQTT_CLIENT_IDX);
    issueCommand(header, NBIOT_TIMEOUT_SEND_MS, CmdKind::QMTPUB, _publishPayload, _publishPayloadLen);
    _publishInFlight = true;
    return;
  }

  CmdOutcome outcome = consumeOutcome();
  if (outcome == CmdOutcome::NONE) return;

  _publishInFlight = false;
  if (outcome == CmdOutcome::OK) {
    onPublishSucceeded();
  } else {
    onPublishFailed();
  }
}

// ---- RPC_REPLY: publicar las respuestas en cola ----
// Publica una respuesta RPC en cola y vuelve directamente a IDLE. A proposito mas
// simple que PUBLISHING: sin reintentos, sin trozos, sin anillo que vaciar. Si falla,
// la respuesta se tira y ThingsBoard deja expirar el RPC por su cuenta: reintentar el
// resultado viejo de una orden es peor que no contestar.
void ModemNBIoTMqtt::tickRpcReply() {
  if (!rpcReplyPending()) {
    setState(State::IDLE);
    return;
  }
  if (!_mqttConnected) {
    popRpcReply();  // no se puede contestar; que ThingsBoard lo deje expirar
    setState(State::IDLE);
    return;
  }
  if (_cmd.active) return;

  RpcReply &r = _rpcReplies[_rpcReplyHead];

  if (!_rpcReplyInFlight) {
    _publishPayloadLen = strlen(r.payload);
    if (_publishPayloadLen >= sizeof(_publishPayload)) _publishPayloadLen = sizeof(_publishPayload) - 1;
    memcpy(_publishPayload, r.payload, _publishPayloadLen);

    char header[128];
    snprintf(header, sizeof(header), "AT+QMTPUB=%d,0,0,0,\"%s\"", MQTT_CLIENT_IDX, r.topic);
    issueCommand(header, NBIOT_TIMEOUT_SEND_MS, CmdKind::QMTPUB, _publishPayload, _publishPayloadLen);
    _rpcReplyInFlight = true;
    return;
  }

  CmdOutcome outcome = consumeOutcome();
  if (outcome == CmdOutcome::NONE) return;

  if (outcome != CmdOutcome::OK) Serial.println("[nbiot-mqtt] rpc reply publish failed - dropped");
  _rpcReplyInFlight = false;
  popRpcReply();
  // Seguir directamente si hay mas esperando - IDLE volveria a entrar aqui de todos
  // modos, pero asi la cola se vacia de una pasada en vez de una por tick.
  setState(rpcReplyPending() ? State::RPC_REPLY : State::IDLE);
}

// ---- ERROR: esperar, apagar el modem de verdad y volver a empezar ----
void ModemNBIoTMqtt::tickError() {
  if (millis() < _backoffUntilMs) return;

  // UN REINTENTO TIENE QUE QUITAR LA ALIMENTACION DE VERDAD, y esto no lo hacia.
  //
  // El bucle era ERROR -> OFF -> enterPowering() -> POWERING -> WAIT_AT -> ERROR.
  // enterPowering() solo ACTIVA las puertas EN y del canal - que ya estaban activas,
  // porque nada en este camino las bajaba - y luego pulsa PWRKEY. powerCycle(), la
  // unica funcion que de verdad corta VIN, no estaba en este camino, asi que
  // _powerCycleSettleUntilMs seguia a 0 y tickOff() pasaba de largo.
  //
  // EN UN MODULO QUECTEL, PWRKEY CONMUTA. Un pulso a un modulo apagado lo enciende; el
  // mismo pulso a un modulo FUNCIONANDO lo APAGA. Asi que cada "reintento" pulsaba
  // PWRKEY sobre un modulo cuyo estado no se conocia y que, tras el primer intento,
  // probablemente estaba encendido - de modo que el reintento tenia tantas
  // posibilidades de apagar el modem como de arrancarlo. Eso explica tambien por que
  // el fallo a veces se arreglaba "solo" tras varios ciclos, y por que afectaba igual
  // a las dos placas: es paridad, no hardware.
  //
  // Cortar VIN primero hace que el pulso de PWRKEY no sea ambiguo: se sabe que el
  // modulo esta apagado, asi que un pulso solo puede significar "enciende".
  // NBIOT_POWER_OFF_SETTLE_MS (4 s) da tiempo a que el rail baje de verdad antes de
  // volver a subir; tickOff() espera al temporizador que arma powerDown().
  powerDown();
  setState(State::OFF);
}

// Publicacion correcta: quitar del anillo lo publicado y seguir si queda algo.
void ModemNBIoTMqtt::onPublishSucceeded() {
  _packetsSent++;
  _flushRequested = false;  // solo una vez que ha salido de verdad
  _consecutiveFailures = 0;
  _publishRetries = 0;
  popSentReadings();
  // Un lote se vacia en varias QMTPUB de MQTT_PUB_CHUNK_READINGS lecturas: se queda
  // en PUBLISHING hasta que el anillo esta vacio, y solo entonces vuelve a IDLE. La
  // sesion ya esta abierta, asi que el siguiente trozo sale en el tick siguiente sin
  // reconectar. Llegan lecturas mucho mas despacio de lo que se vacian (un trozo por
  // tick), asi que esto siempre termina.
  if (_ringCount > 0) {
    _publishInFlight = false;
    return;
  }
  setState(State::IDLE);
}

void ModemNBIoTMqtt::onPublishFailed() {
  _packetsFailed++;
  handleFailureAtLevel(RecoveryLevel::PUBLISH, "publish failed");
}

// Escribe en out las cifras decimales de un int64 no negativo (sin '\0') y devuelve
// cuantas son. Evita depender de que %lld este compilado en el snprintf de este
// compilador (la nano newlib a menudo no lo trae).
static size_t appendI64(char *out, int64_t v) {
  if (v < 0) v = 0;  // aqui la epoca en ms siempre es positiva - solo por precaucion
  char tmp[24];
  size_t n = 0;
  uint64_t u = (uint64_t)v;
  do {
    tmp[n++] = (char)('0' + (int)(u % 10));
    u /= 10;
  } while (u);
  for (size_t i = 0; i < n; i++) out[i] = tmp[n - 1 - i];
  return n;
}

// ---------- El JSON de telemetria para la API de pasarela de ThingsBoard ----------
//
// Construye un mensaje de telemetria de pasarela con hasta maxReadings de las
// lecturas pendientes mas antiguas del anillo:
//   {"<nodo>":[<registro>,<registro>,...], "<otroNodo>":[...]}
// Las lecturas se agrupan por nombre de nodo (ver nbiotResolveNodeName en Config.h)
// en vez de suponer un solo nodo.
//
// Dos formas de registro, y la eleccion no es libre - el interprete de pasarela de
// ThingsBoard solo acepta el envoltorio "values" si va con un "ts":
//   _haveNetTime : {"ts":<epoca_ms>,"values":{<claves>}}  - hora real de cada lectura
//   si no        : {<claves>}                             - el servidor pone la hora al recibir
// El ts de cada lectura es el momento en que se TOMO: netNowMs() trasladado a traves
// del lastHeardMs de la propia lectura (un valor de millis() ya corregido por su
// antiguedad), asi que un lote de lecturas espaciadas llega a ThingsBoard como una
// serie real, no como varios puntos en el mismo instante. Sin reloj de red se guarda
// igual, solo que con la resolucion de la llegada del lote.
size_t ModemNBIoTMqtt::buildGatewayPayload(uint8_t *out, size_t cap, uint8_t maxReadings) {
  char *buf = reinterpret_cast<char *>(out);
  size_t pos = 0;
  auto append = [&](const char *s) {
    size_t n = strlen(s);
    if (pos + n < cap) {
      memcpy(buf + pos, s, n);
      pos += n;
    }
  };

  uint8_t count = _ringCount;
  if (count > maxReadings) count = maxReadings;

  bool grouped[NBIOT_RING_CAPACITY] = {false};
  char groupName[24];

  append("{");
  bool firstNode = true;
  for (uint8_t i = 0; i < count; i++) {
    if (grouped[i]) continue;
    nbiotResolveNodeName(ringPeek(i).senderAddr, groupName, sizeof(groupName));

    if (!firstNode) append(",");
    firstNode = false;
    append("\"");
    append(groupName);
    append("\":[");

    bool firstReading = true;
    for (uint8_t j = i; j < count; j++) {
      if (grouped[j]) continue;
      const SensorSnapshot &snap = ringPeek(j);
      char thisName[24];
      nbiotResolveNodeName(snap.senderAddr, thisName, sizeof(thisName));
      if (strcmp(thisName, groupName) != 0) continue;

      grouped[j] = true;
      if (!firstReading) append(",");
      firstReading = false;

      // UN CAMPO QUE EL NODO NO MIDIO NO VIAJA.
      //
      // El nodo manda -999 en los campos que esa lectura concreta no midio. El
      // CM1106 va cada 30 min y el VOC del BME690 cada 20, asi que la mayoria de
      // filas no llevan CO2 ni gas. Antes el nodo repetia el ultimo valor
      // conocido, lo que hacia INDISTINGUIBLE "el CO2 no se movio" de "nadie miro
      // el CO2" - el mismo fallo que el viento congelado y el co2=663 fantasma de
      // C1, que costo dias de no entender nada.
      //
      // Y reenviar el -999 seria aun peor que repetir: la grafica lo pintaria como
      // una medida de -999 ppm. Se OMITE la clave, que es lo que de verdad
      // ocurrio, y ThingsBoard lo trata bien - no registra punto para esa serie en
      // ese instante.
      char keys[500];  // +3 claves para chargePct/capMv/alarmState
      int kn = 0;
      keys[kn++] = '{';
      keys[kn] = '\0';
#define SIN_MEDIR_B(v) ((v) < -900.0f)
#define CLAVE_F(nombre, valor)                                                        \
  do {                                                                                \
    if (!SIN_MEDIR_B(valor)) {                                                        \
      kn += snprintf(keys + kn, sizeof(keys) - (size_t)kn, "%s\"" nombre "\":%.2f",    \
                     (kn > 1) ? "," : "", (double)(valor));                            \
    }                                                                                 \
  } while (0)

      CLAVE_F("temp", snap.temp);
      CLAVE_F("rh", snap.hum);
      CLAVE_F("pres", snap.pres);
      CLAVE_F("gas", snap.gas);
      CLAVE_F("pm1", snap.pm1);
      CLAVE_F("pm25", snap.pm25);
      CLAVE_F("pm10", snap.pm10);
      CLAVE_F("co2", snap.co2);
      CLAVE_F("co", snap.co);
      CLAVE_F("coTemp", snap.coTemp);
      CLAVE_F("windAngle", snap.windAngle);
      CLAVE_F("windSpeed", snap.windSpeed);
#undef CLAVE_F
#undef SIN_MEDIR_B

      // Estos van SIEMPRE: no son medidas de sensor sino el estado del enlace y de
      // la placa, y ahi la ausencia si seria una perdida de informacion.
      kn += snprintf(keys + kn, sizeof(keys) - (size_t)kn,
                     "%s\"windValid\":%s,\"rssi\":%d,\"snr\":%d,"
                     "\"chargePct\":%d,\"capMv\":%d,\"alarmState\":%d}",
                     (kn > 1) ? "," : "", snap.windValid ? "true" : "false",
                     (int)snap.rssi, (int)snap.snr, (int)snap.chargePct, (int)snap.capMv,
                     (int)snap.alarmState);

      if (_haveNetTime) {
        int64_t ts = _netEpochMsAtSync + (int64_t)(int32_t)(snap.lastHeardMs - _netSyncLocalMs);
        char tsField[40];
        size_t tn = 0;
        memcpy(tsField, "{\"ts\":", 6);
        tn = 6;
        tn += appendI64(tsField + tn, ts);
        memcpy(tsField + tn, ",\"values\":", 10);
        tn += 10;
        tsField[tn] = '\0';
        append(tsField);
        append(keys);
        append("}");
      } else {
        append(keys);
      }
    }
    append("]");
  }
  append("}");

  _lastPublishCount = count;
  return pos;
}
