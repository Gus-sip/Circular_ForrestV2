#pragma once

#include <Arduino.h>

#include "Config.h"
#include "nbiot/NbiotProtocol.h"
#include "telemetry/SensorSnapshot.h"

// =============================================================================
// DRIVER DEL MODEM NB-IoT (Quectel BC660K-GL) CON MQTT HACIA THINGSBOARD
// =============================================================================
//
// Driver NO BLOQUEANTE del cliente MQTT integrado en el BC660K-GL, que publica la
// telemetria de los nodos en ThingsBoard mediante la API de pasarela (Gateway API).
// Es hermano de ModemNBIoT (la via antigua por UDP), no un reemplazo dentro de el:
// el transporte cambia demasiado (QMTOPEN/QMTCONN encadenados sin pausa, una
// publicacion con longitud + aviso '>' + Ctrl-Z, JSON en vez de una trama binaria,
// otros avisos) como para cambiar un solo metodo de ModemNBIoT.
//
// La secuencia de encendido y registro (de POWERING a ATTACHING) esta confirmada en
// el hardware desde el 2026-08-19, y es distinta de la que hacia ModemNBIoT (que
// nunca se valido: pines equivocados, sin canal ni PWRKEY, AT+CFUN=1 despues de
// AT+CPIN? en vez de antes, AT+CGDCONT en vez de AT+QCGDEFCONT). Solo se corrigio
// aqui, porque aquella clase se esta retirando.
//
// Maquina de estados:
//   OFF -> POWERING -> WAIT_AT -> CONFIG -> ATTACHING -> MQTT_CONNECT -> IDLE
//   IDLE <-> PUBLISHING (lote listo; vuelve a IDLE en cualquier caso)
//   IDLE <-> RPC_REPLY (respuestas a ThingsBoard pendientes)
//   cualquier estado -> ERROR ante un plazo agotado sin arreglo; ERROR -> OFF tras
//   una espera creciente (backoff).
//
// Politica de la sesion MQTT: se mantiene abierta entre envios y no se le hace ping
// por iniciativa propia entre lotes (ver MQTT_KEEPALIVE_S en Config.h). Si
// PUBLISHING encuentra la sesion caida (nunca abierta, o +QMTSTAT dijo que se cayo),
// pasa antes por MQTT_CONNECT y despues publica. Ademas IDLE la reconstruye por si
// sola si se cae sin que haya nada que publicar (ver tickIdle()).
//
// Recuperacion escalonada ante fallos: reintentar la publicacion, despues
// reconectar MQTT (QMTOPEN+QMTCONN otra vez), despues volver a registrarse en la
// red, y solo como ultimo recurso apagar y encender el modem.
//
// En sentido contrario (de ThingsBoard a los nodos) recibe RPC por MQTT, los
// guarda en una cola por nodo, y main.cpp los entrega por LoRa en la ventana de
// escucha de cada nodo; el resultado (ACK/NACK/timeout) se devuelve a ThingsBoard.
class ModemNBIoTMqtt {
public:
  enum class State : uint8_t {
    OFF,
    POWERING,
    WAIT_AT,
    CONFIG,
    ATTACHING,
    MQTT_CONNECT,
    IDLE,
    PUBLISHING,
    RPC_REPLY,
    ERROR,
  };

  ModemNBIoTMqtt(HardwareSerial &serial, uint8_t rxPin, uint8_t txPin, uint8_t enPin, uint8_t channelPin,
                 uint8_t pwrkeyPin, uint32_t baud = NBIOT_BAUD);

  // Configura los pines y deja el modem apagado. Se llama una vez desde setup().
  void begin();

  // Hace avanzar la maquina de estados: una pequena parte acotada de trabajo y
  // vuelve, sin bloquear nunca. Se llama en cada vuelta de loop().
  void tick();

  // Mete una lectura en el anillo de lotes. Nunca bloquea. Si el anillo ya esta
  // lleno, se descarta la mas antigua y sube packetsDropped().
  void enqueue(const SensorSnapshot &snap);

  // Publicar en la primera ocasion en vez de esperar al lote.
  //
  // Los lotes existen para que el enlace NB-IoT y los datos de la SIM sean
  // razonables, y para la telemetria es lo correcto. Un incendio no es telemetria:
  // un nodo informando de uno mientras Modulo B se queda el paquete el resto de una
  // ventana de 60 s es un minuto de silencio justo en el momento para el que existe
  // el sistema.
  void requestFlush() { _flushRequested = true; }

  // Publica un registro commandLog: el resultado de una orden que ya se confirmo
  // como "enviado". Usa la misma cola de publicacion que las respuestas RPC.
  void publishCommandLog(long requestId, const char *status);

  // El mismo registro mas un campo "detail": las claves que el nodo aplico de verdad,
  // o el motivo por el que lo rechazo. El documento del protocolo define
  // requestId/status/timestamp; esto anade sin cambiar, asi que un panel que lea solo
  // los campos documentados no se ve afectado.
  // `device` ademas copia el registro en la telemetria de ese nodo, para que se pueda
  // leer tanto si el panel mira la pasarela como si mira el nodo.
  void publishCommandLogApplied(long requestId, const char *status, const char *detail,
                                const char *device = nullptr);

private:
  void handleConfigurarModuloC(const NbiotProtocol::MqttMessage &msg, const char *idStr);

public:

  // Pone el estado de incendio ACTUAL de un nodo en las lecturas suyas que ya estan
  // esperando a salir.
  //
  // Sin esto, la publicacion urgente por alarma subiria un estado viejo, y seria puro
  // teatro. Un nodo transmite PRIMERO sus lecturas y su STAT unos dos segundos
  // despues, asi que cuando Modulo B se entera de que el nodo ha entrado en alarma,
  // la lectura de ese ciclo ya esta en cola con el estado ANTERIOR. requestFlush()
  // sacaria deprisa un paquete diciendo alarmState 0 - justo el paquete cuyo unico
  // proposito era decir lo contrario - y el estado real llegaria en el lote
  // siguiente, hasta un minuto despues.
  //
  // Confirmado en el hardware el 2026-09-28: los tres nodos publicaron alarmState -1
  // en un lote y 0 en el siguiente, siempre un ciclo por detras de sus STAT.
  void applyAlarmToQueued(uint16_t addr, int8_t state) {
    for (uint8_t i = 0; i < _ringCount; i++) {
      if (_ring[i].senderAddr == addr) _ring[i].alarmState = state;
    }
  }

  // ---------- Observabilidad: lo que muestran el panel y la OLED ----------
  State state() const { return _state; }
  // El UART del modem NO se mantiene abierto a traves de los apagados, a proposito:
  // un pin TX conducido alimenta el modulo por la puerta de atras a traves de sus
  // diodos de proteccion e impide que arranque en frio. Ver enterPowering().
  void releaseUart();
  void startUart();

  const char *stateName() const;
  // Nombra un estado sin necesitar una instancia en ese estado: setState() tiene que
  // poder imprimir el estado al que ENTRA antes de asignar _state.
  static const char *stateNameOf(State st);
  const char *lastError() const { return _lastError; }
  int rssiDbm() const { return _rssiDbm; }
  bool attached() const { return _attached; }
  uint32_t attachedUptimeMs() const;
  bool mqttConnected() const { return _mqttConnected; }
  uint32_t packetsSent() const { return _packetsSent; }
  uint32_t packetsFailed() const { return _packetsFailed; }
  uint32_t packetsDropped() const { return _packetsDropped; }
  uint32_t mqttReconnects() const { return _mqttReconnects; }
  uint16_t batchReadingsTarget() const { return _batchReadingsTarget; }
  uint32_t batchSecondsTarget() const { return _batchSecondsTarget; }
  uint8_t ringCount() const { return _ringCount; }
  bool haveNetTime() const { return _haveNetTime; }

  // Epoca UNIX en SEGUNDOS, para bajarsela a los nodos por CFG,TIME. Devuelve 0
  // cuando la red no ha dado hora todavia, y el llamante tiene que tratar ese 0
  // como "no se" y no mandar nada - un cero situaria al nodo en 1970.
  //
  // EN SEGUNDOS Y NO EN MILISEGUNDOS a proposito: cabe en 32 bits hasta 2106, asi
  // que ni este extremo ni el nodo necesitan enteros de 64 bits ni `%lld`, que no
  // es de fiar con la nano-newlib de este build - ya nos costo un fallo aqui
  // mismo. Se pierde precision por debajo del segundo, irrelevante contra
  // periodos de minutos.
  unsigned long netEpochSeconds() const {
    if (!_haveNetTime) return 0;
    const int64_t ms = netNowMs();
    return (ms > 0) ? (unsigned long)(ms / 1000) : 0;
  }

  // ---------- Configuracion en marcha, ajustable desde Modulo A ----------
  // Las dos recortan a los limites MQTT_BATCH_*_MIN/MAX de Config.h y devuelven el
  // valor realmente aplicado, para que un recorte sea visible (y se devuelva en la
  // respuesta RPC) en vez de darse por supuesto.
  uint32_t setBatchSeconds(uint32_t seconds);

  bool _flushRequested = false;  // ver requestFlush()
  uint16_t setBatchReadings(uint16_t readings);

  // ---------- Reenvio de ordenes a un nodo hijo ----------
  // Una orden dirigida a un nodo no se puede entregar al llegar: Modulo C solo
  // escucha ~2 s despues de cada transmision suya. Asi que se aparca aqui y main.cpp
  // la manda por LoRa en cuanto ve la subida de ese nodo.
  //
  // UNA LISTA DE PENDIENTES, como especifica el documento del protocolo ("guardar
  // comando en lista pendiente"), no un unico hueco.
  //
  // Un nodo solo oye en la ventana de ~2 s tras su subida, asi que una orden puede
  // esperar minutos a entregarse. Con un solo hueco, todas las ordenes que llegaban
  // durante esa espera se rechazaban como "ocupado" - y el panel de Modulo A tiene
  // una fila por magnitud, asi que poner la temperatura y luego la humedad hacia que
  // la segunda se rechazara por un motivo que el operador no podia ver ni resolver.
  //
  // Todo va por dispositivo: una orden para C-2 no debe bloquear nunca una para C-3
  // solo porque C-2 aun no se ha despertado.
  const char *nodeCommandCfgFor(const char *device) const {
    const int i = deliverableIndexFor(device);
    return i < 0 ? nullptr : _nodeCmds[i].cfg;
  }
  // main.cpp la llama cuando la cadena CFG de ese dispositivo ya ha salido por LoRa.
  void onNodeCommandDelivered(const char *device);
  // main.cpp la llama cuando vuelve un paquete "ACK,..." de un nodo. Informa del
  // resultado y libera el hueco de ese dispositivo.
  void onNodeCommandAck(const char *device, const char *ackPayload);

  // El nodo entendio la orden y no aplico NADA. Se informa enseguida como el estado
  // "error" del protocolo, en vez de dejar que expire como si el nodo fuera
  // inalcanzable.
  void onNodeCommandNack(const char *device, const char *reason);

private:
  // Tipo de comando AT en curso (los QMT* terminan con un aviso asincrono).
  enum class CmdKind : uint8_t { PLAIN, QMTOPEN, QMTCONN, QMTPUB, QMTSUB };
  enum class CmdOutcome : uint8_t { NONE, OK, ERR, TIMEOUT };
  // Pasos del estado CONFIG, en orden.
  enum class ConfigStep : uint8_t {
    ATE0,
    CMEE,
    CTZU,  // AT+CTZU=1 - actualizacion automatica de la hora de red, para que AT+CCLK? funcione (si se puede)
    QSCLK,
    CFUN_ON_1,
    CPIN,
    CFUN_OFF,
    QCGDEFCONT,
    CFUN_ON_2,
    DONE
  };
  // Pasos del estado MQTT_CONNECT, en orden.
  //
  // SUB_DEVICE/SUB_GATEWAY van despues de CONN para que la bajada de ordenes este
  // activa antes de IDLE. Las dos son "si se puede": una suscripcion fallida pierde
  // el control remoto pero no debe parar la telemetria, que es el trabajo real del
  // modulo.
  //
  // ANNOUNCE va el ultimo, tras las dos suscripciones, y no es opcional.
  //
  // ThingsBoard no encamina un RPC a un dispositivo hijo de una pasarela hasta que la
  // pasarela lo ha anunciado en v1/gateway/connect. La telemetria NO lo necesita -
  // publicar en v1/gateway/telemetry crea el dispositivo solo -, y por eso el olvido
  // era invisible: los nodos aparecian en ThingsBoard con lecturas en vivo mientras
  // todas las ordenes que se les mandaban morian en el servidor, antes de llegar al
  // modem.
  enum class MqttConnectSub : uint8_t {
    CLOSE_FIRST, KEEPALIVE_CFG, SESSION_CFG, OPEN, CONN, SUB_DEVICE, SUB_GATEWAY, ANNOUNCE
  };
  // Hasta donde retroceder al recuperarse de un fallo.
  enum class RecoveryLevel : uint8_t { PUBLISH, MQTT_CONNECT, ATTACH };

  // El comando AT en curso (solo hay uno a la vez).
  struct PendingCmd {
    bool active = false;
    CmdKind kind = CmdKind::PLAIN;
    uint32_t sentAtMs = 0;
    uint32_t timeoutMs = 0;
    bool sawOk = false;           // QMTOPEN/QMTCONN/QMTPUB: ya llego el OK inmediato, se espera el aviso asincrono
    bool awaitingPrompt = false;  // QMTPUB: esperando el '>' que pide los datos
    const uint8_t *sendPayload = nullptr;
    size_t sendPayloadLen = 0;
    char infoLine[96] = {0};      // la linea de respuesta con datos (p. ej. "+CSQ: 20,99")
    bool hasInfoLine = false;
    CmdOutcome outcome = CmdOutcome::NONE;
  };

  // ---------- El unico camino de los comandos ----------
  void issueCommand(const char *cmd, uint32_t timeoutMs, CmdKind kind = CmdKind::PLAIN,
                     const uint8_t *sendPayload = nullptr, size_t sendPayloadLen = 0);
  void pumpSerial();
  void handleLine(const char *line, size_t len);
  void handleUrc(const char *line, size_t len);
  void completeCommand(CmdOutcome outcome);
  void checkCommandTimeout();
  CmdOutcome consumeOutcome();

  // ---------- Maquina de estados ----------
  void setState(State s);
  void enterPowering();
  void enterError(const char *reason);
  void powerCycle();

  // La mitad electrica de un apagado y encendido: TX liberado, VIN y la puerta del
  // canal cortados, PWRKEY en reposo, y el temporizador de espera en apagado armado.
  // La comparten powerCycle() y el camino de reintento ERROR->OFF, que TIENE que
  // quitar la alimentacion en vez de solo volver a pulsar PWRKEY - ver tickError().
  void powerDown();
  void handleFailureAtLevel(RecoveryLevel level, const char *reason);
  void applyBackoffAndSetState(State s);
  uint32_t computeBackoff(uint32_t attempt) const;
  void setLastError(const char *msg);

  // ---------- Ordenes recibidas ----------
  // Reparte un mensaje MQTT entrante segun su topic: el topic de RPC propio de la
  // pasarela significa "reconfigurar Modulo B", y el topic de RPC de pasarela
  // significa "reenviar esto a un nodo hijo".
  void handleInboundMqtt(const NbiotProtocol::MqttMessage &msg);
  void handleOwnRpc(const NbiotProtocol::MqttMessage &msg);
  void handleGatewayRpc(const NbiotProtocol::MqttMessage &msg);
  // Aplica un objeto plano {"CLAVE":valor,...} a la configuracion en marcha de
  // Modulo B, escribiendo en applied lo realmente aplicado (tras recortar).
  bool applyOwnConfig(const char *params, char *applied, size_t cap);
  // Pone en cola una respuesta RPC para que la publique RPC_REPLY.
  void queueRpcReply(const char *topic, const char *payload);
  void tickNodeCommandTimeout();

  // Un tick por estado.
  void tickOff();
  void tickPowering();
  void tickWaitAt();
  void tickConfig();
  void tickAttaching();
  void tickMqttConnect();
  void tickIdle();
  void tickPublishing();
  void tickRpcReply();
  void tickError();

  void onPublishSucceeded();
  void onPublishFailed();

  // ---------- Anillo de lecturas ----------
  const SensorSnapshot &ringPeek(uint8_t i) const;
  void popSentReadings();
  // Construye el contenido de un AT+QMTPUB con hasta maxReadings de las lecturas mas
  // antiguas del anillo (agrupadas por nombre de nodo). Pone en _lastPublishCount
  // cuantas metio de verdad. Con hora de red emite {"ts":<ms>,"values":{...}} por
  // lectura; sin ella, la forma plana sin "ts".
  size_t buildGatewayPayload(uint8_t *out, size_t cap, uint8_t maxReadings);

  // "Ahora" en ms de epoca, a partir de la ultima sincronizacion con AT+CCLK? mas el
  // millis() transcurrido. Solo tiene sentido con _haveNetTime.
  int64_t netNowMs() const;

  // ---------- Configuracion fija ----------
  HardwareSerial &_serial;
  uint8_t _rxPin, _txPin, _enPin, _channelPin, _pwrkeyPin;
  uint32_t _baud;
  bool _uartStarted = false;

  // ---------- Motor de comandos ----------
  PendingCmd _cmd;
  // TIENE QUE CABER UNA LINEA +QMTRECV ENTERA, con su contenido.
  //
  // Valia 96 y rompia en silencio todas las ordenes. Una linea RPC de pasarela es
  //
  //   +QMTRECV: 0,0,"v1/devices/me/rpc/request/123",{...json...}
  //
  // y solo el JSON de la orden de umbral pasa de 150 caracteres, asi que la linea
  // era siempre mas larga que el buffer. Peor que cortarla: el camino de
  // desbordamiento vaciaba el buffer y seguia, asi que se descartaba el PRINCIPIO
  // del mensaje y el FINAL se pasaba al interprete como si fuera una linea completa.
  // parseQmtrecv fallaba con el, y los restos aparecian como
  //
  //   [nbiot-mqtt] urc: n":50,"umbral_alarm_off":55,"umbral_alarm_on":60}}"
  //
  // que parece ruido y es en realidad el final de una orden. Todas las conclusiones
  // de "no llega nada" durante varios dias salian de buscar el mensaje de exito,
  // que solo se imprime cuando la interpretacion funciona.
  //
  // Dimensionado para lo mas grande que puede llegar: topic (72) + contenido (320)
  // mas el envoltorio de +QMTRECV, redondeado hacia arriba.
  static const size_t kLineBufLen = 512;
  char _lineBuf[kLineBufLen];
  bool _lineOverflowed = false;
  uint8_t _lineLen = 0;
  bool _sawBootUrc = false;

  // ---------- Contabilidad de la maquina de estados ----------
  State _state = State::OFF;
  uint32_t _stateEnteredMs = 0;
  uint32_t _backoffUntilMs = 0;
  uint32_t _powerCycleSettleUntilMs = 0;
  uint32_t _consecutiveFailures = 0;
  char _lastError[64] = "none";

  bool _pwrkeyReleased = false;  // tiempos dentro de POWERING: pulsar a bajo y soltar a alto, sin bloquear
  uint32_t _lastAtPingMs = 0;

  ConfigStep _configStep = ConfigStep::ATE0;
  uint32_t _configStepEnteredMs = 0;  // deja a QCGDEFCONT esperar a que se asiente la SIM tras CFUN=0, sin bloquear

  bool _cgpaddrChecked = false;
  bool _csqCheckedAfterAttach = false;  // un AT+CSQ justo tras el registro - ver tickAttaching()
  bool _cclkChecked = false;            // un AT+CCLK? justo tras el de CSQ - ver tickAttaching()
  uint32_t _lastCeregPollMs = 0;
  uint8_t _attachRetries = 0;

  // ---------- Hora de red (marca de tiempo de cada lectura subida) ----------
  // Se lee una vez por registro con AT+CCLK? (necesita AT+CTZU=1). _netEpochMsAtSync
  // es la epoca en ms en el momento en que se tomo _netSyncLocalMs (un valor de
  // millis()); netNowMs() extrapola desde ahi. Se queda en "sin hora" si la red no
  // da nunca un reloj creible, y entonces la subida no lleva "ts".
  bool _haveNetTime = false;
  int64_t _netEpochMsAtSync = 0;
  uint32_t _netSyncLocalMs = 0;

  MqttConnectSub _mqttConnectSub = MqttConnectSub::KEEPALIVE_CFG;
  uint8_t _mqttConnectRetries = 0;
  uint8_t _announceIdx = 0;  // que nodo se esta anunciando en v1/gateway/connect
  bool _mqttConnected = false;
  uint32_t _mqttReconnects = 0;

  uint32_t _lastCsqPollMs = 0;  // refresco del RSSI para el panel mientras esta en IDLE

  bool _publishInFlight = false;
  uint8_t _publishRetries = 0;
  uint8_t _publishPayload[MQTT_PAYLOAD_MAX_BYTES];
  size_t _publishPayloadLen = 0;
  uint8_t _lastPublishCount = 0;

  // ---------- Cola de ordenes para los nodos ----------
  struct NodeCommand {
    bool pending = false;      // hueco ocupado
    bool awaitingAck = false;  // ya enviada por LoRa, esperando el ACK del nodo
    char device[24] = {0};     // nombre del dispositivo hijo en ThingsBoard, p. ej. "NodoC-1"
    char cfg[128] = {0};       // "CFG,INTERVAL=300,BMV080=0" tal como lo espera Modulo C
    char supKey[24] = {0};     // que AJUSTE toca, para que un comando sustituya al anterior
    long rpcId = 0;
    uint32_t queuedMs = 0;
    uint8_t attempts = 0;      // intentos de entrega hasta ahora - se informa, y se reintenta
    uint32_t lastSentMs = 0;   // cuando salio el ultimo intento, para el hueco entre reintentos

    // Que protocolo la pidio, y por tanto como se informa su RESULTADO.
    //
    //   false - el camino de RPC de pasarela: el RPC se deja abierto y se contesta con
    //           el resultado, asi que ThingsBoard espera hasta que el nodo responde o
    //           se agota el plazo.
    //   true  - el protocolo A->C acordado: el RPC ya se contesto "enviado" en cuanto
    //           llego, y el resultado sale despues como un registro de telemetria
    //           commandLog con el mismo requestId.
    //
    // La distincion tiene que ir en la propia orden: cuando llegan el ACK o el plazo
    // agotado, el mensaje original hace mucho que no existe.
    bool viaCommandLog = false;
  };
  // Seis: suficiente para todas las magnitudes de un nodo, o dos por nodo en tres
  // nodos. Una cola llena se INFORMA, nunca se tira en silencio.
  // La cadena CFG se construye aqui antes de elegir un hueco libre, para que una orden
  // que al final no se puede encolar no deje escrito a medias un hueco real.
  char _pendingCfgScratch[128] = {0};

  static const uint8_t kNodeCmdQueue = 6;
  NodeCommand _nodeCmds[kNodeCmdQueue];

  // Hueco de este dispositivo aun no enviado (o -1).
  int undeliveredIndexFor(const char *device) const {
    for (uint8_t i = 0; i < kNodeCmdQueue; i++) {
      if (_nodeCmds[i].pending && !_nodeCmds[i].awaitingAck &&
          strcmp(_nodeCmds[i].device, device) == 0) {
        return (int)i;
      }
    }
    return -1;
  }

  // UN COMANDO NUEVO SUSTITUYE AL ANTERIOR DEL MISMO NODO Y EL MISMO AJUSTE.
  //
  // Sin esto, seis ordenes identicas al mismo nodo ocupan los seis huecos y la
  // cola se bloquea, aunque la primera ya haga el trabajo. Pasa constantemente:
  // el operador repite el comando al no ver respuesta, y ademas ThingsBoard
  // guarda hasta una hora los RPC no entregados y se los descarga a Modulo B de
  // golpe al reconectar, llenando la cola en el arranque.
  //
  // El propio documento v2 ya establece el principio para NxT_enviodatos: "Manda
  // el ultimo recibido". Se aplica a todas las acciones, que es lo coherente -
  // si llegan dos umbrales distintos para la misma variable, el que vale es el
  // ultimo que mando el operador, no el primero que entro en la cola.
  //
  // Solo sustituye comandos que AUN NO SE HAN ENVIADO. Uno ya entregado y
  // esperando ACK se deja en paz: el nodo ya lo tiene, y su resultado hay que
  // reportarlo.
  int supersedableIndexFor(const char *device, const char *supKey) const {
    for (uint8_t i = 0; i < kNodeCmdQueue; i++) {
      const NodeCommand &nc = _nodeCmds[i];
      if (!nc.pending || nc.awaitingAck) continue;
      if (strcmp(nc.device, device) != 0) continue;
      if (strcmp(nc.supKey, supKey) != 0) continue;
      return (int)i;
    }
    return -1;
  }

  // QUE AJUSTE toca este comando, derivado del CFG que se va a mandar al nodo.
  //
  // Agrupar por `action` era un error: TODOS los umbrales comparten la accion
  // `umbrales_de1variable`, asi que configurar temp y despues rh en el mismo nodo
  // cancelaba el primero y lo reportaba como error. La cola esta dimensionada "lo
  // suficiente para cada magnitud de un nodo" precisamente para que eso no pase.
  //
  // Se deriva del CFG y no de la accion porque el CFG es lo que de verdad va a
  // cambiar en el nodo, y eso agrupa bien dos casos que por accion no coinciden:
  // `umbral` y `umbrales_de1variable` son alias, y `NxT_enviodatos` y
  // `NxTglobalenviodatos` producen el mismo CFG,NXT_ENVIO. Dos nombres para un
  // mismo ajuste no deben ocupar dos huecos.
  //
  //   "CFG,temp_pre_off=44,temp_pre_on=50"  ->  "temp"
  //   "CFG,NXT_ENVIO=0.5"                   ->  "NXT_ENVIO"
  //   "CFG,INTERVAL=300"                    ->  "INTERVAL"
  static void supersedeKeyFromCfg(const char *cfg, char *out, size_t cap) {
    out[0] = 0;
    const char *p = strchr(cfg, ',');       // saltar el "CFG"
    if (!p) return;
    p++;
    const char *eq = strchr(p, '=');
    if (!eq) return;

    size_t len = (size_t)(eq - p);
    if (len >= cap) len = cap - 1;
    memcpy(out, p, len);
    out[len] = 0;

    // Un umbral se nombra <magnitud><sufijo>. Recortar el sufijo deja la
    // magnitud, que es el ajuste real: dos ordenes sobre `temp` se sustituyen,
    // una sobre `temp` y otra sobre `rh` no.
    static const char *kSuf[] = {"_pre_off", "_pre_on", "_alarma_off", "_alarma_on",
                                 "_alarm_off", "_alarm_on"};
    for (size_t i = 0; i < sizeof(kSuf) / sizeof(kSuf[0]); i++) {
      const size_t sl = strlen(kSuf[i]);
      const size_t ol = strlen(out);
      if (ol > sl && strcmp(out + ol - sl, kSuf[i]) == 0) {
        out[ol - sl] = 0;
        return;
      }
    }
  }

  // Cuantos huecos quedan. Una accion global necesita uno POR NODO, y hay que
  // saberlo ANTES de encolar nada: encolar la mitad de un comando que va a todos
  // los nodos deja el sistema diciendo que lo aplico cuando no es cierto.
  uint8_t freeNodeCmdSlots() const {
    uint8_t n = 0;
    for (uint8_t i = 0; i < kNodeCmdQueue; i++) {
      if (!_nodeCmds[i].pending) n++;
    }
    return n;
  }

  // UN HUECO QUE A ESTE NODO HAY QUE MANDARLE AHORA - lo que incluye uno ya enviado
  // y aun sin confirmar.
  //
  // Que el nodo haga una subida es en si la prueba de que un intento anterior fallo:
  // si hubiera recibido y aplicado la orden, la habria confirmado dentro de esa
  // ventana y el hueco ya estaria libre. Asi que una orden sin confirmar se vuelve a
  // mandar en la siguiente ventana en vez de esperar al plazo.
  //
  // Es lo que siempre se documento que significaba el plazo de 15 minutos - "3
  // ventanas perdidas a 5 min" - salvo que nada reintentaba en esas ventanas. Una
  // trama perdida costaba los quince minutos enteros y luego informaba de un timeout
  // para una orden que se habia entregado a un nodo sano. Visto con el requestId 7,
  // 2026-10-05.
  //
  // La guarda del hueco entre reintentos es esencial: ver DOWNLINK_RETRY_MIN_GAP_MS.
  int deliverableIndexFor(const char *device) const {
    // UNA ORDEN POR NODO Y POR VENTANA.
    //
    // main.cpp ejecutaba este gancho con cada trama recibida, y un nodo manda varias
    // por ventana - telemetria, su ACK y el STAT. Mirar solo si ESTE hueco se habia
    // mandado hacia poco hacia que la segunda orden en cola saliera con la trama ACK
    // del nodo y la tercera con su STAT, cuando el nodo ya habia cerrado su ventana y
    // estaba transmitiendo. Las dos se perdian y habia que reintentarlas una ventana
    // despues.
    //
    // Visto con cinco ordenes encoladas a la vez el 2026-10-05: se vaciaba una por
    // ventana de todos modos, pero gastando dos tramas cada vez.
    //
    // Asi que si se mando ALGO a este nodo dentro del hueco, el nodo esta
    // transmitiendo y sordo: retener el resto.
    for (uint8_t i = 0; i < kNodeCmdQueue; i++) {
      const NodeCommand &nc = _nodeCmds[i];
      if (!nc.pending || strcmp(nc.device, device) != 0) continue;
      if (nc.awaitingAck && millis() - nc.lastSentMs < DOWNLINK_RETRY_MIN_GAP_MS) return -1;
    }

    // LA MAS ANTIGUA PRIMERO, para que las ordenes lleguen al nodo en el orden en que
    // las mando Modulo A. Los huecos se reparten por el primero libre, asi que el
    // orden de los huecos no es el de llegada - y cuando dos ordenes ponen el mismo
    // umbral a valores distintos, tiene que ganar la ultima que se mando. Se usa la
    // edad y no la marca en bruto, para que el desbordamiento de millis() compare
    // bien.
    int best = -1;
    uint32_t bestAge = 0;
    const uint32_t now = millis();
    for (uint8_t i = 0; i < kNodeCmdQueue; i++) {
      const NodeCommand &nc = _nodeCmds[i];
      if (!nc.pending || strcmp(nc.device, device) != 0) continue;
      const uint32_t age = now - nc.queuedMs;
      if (best < 0 || age > bestAge) {
        best = (int)i;
        bestAge = age;
      }
    }
    return best;
  }

  // Hueco de este dispositivo ya enviado y esperando ACK (o -1).
  int awaitingAckIndexFor(const char *device) const {
    for (uint8_t i = 0; i < kNodeCmdQueue; i++) {
      if (_nodeCmds[i].pending && _nodeCmds[i].awaitingAck &&
          strcmp(_nodeCmds[i].device, device) == 0) {
        return (int)i;
      }
    }
    return -1;
  }

  // Primer hueco libre (o -1).
  int freeNodeCmdSlot() const {
    for (uint8_t i = 0; i < kNodeCmdQueue; i++) {
      if (!_nodeCmds[i].pending) return (int)i;
    }
    return -1;
  }

  // ---------- Cola de respuestas a ThingsBoard ----------
  // UNA COLA, NO UN HUECO.
  //
  // Era un unico hueco que apuntaba "descartando la anterior" y tiraba un mensaje. No
  // habia pasado aun solo por suerte - el modem vaciaba cada respuesta antes de que
  // llegara la siguiente - pero que lleguen dos ordenes en el mismo tick es normal
  // (las peticiones 9 y 10 lo hicieron el 2026-10-05), y la descartada habria sido un
  // "enviado" que Modulo A estaba esperando.
  //
  // Ocho de fondo. Cada resultado produce ahora TRES publicaciones - commandLog en la
  // pasarela (lo que especifica el documento del protocolo), el mismo registro
  // copiado en el nodo, y los atributos del nodo - y pueden llegar dos ordenes en un
  // tick.
  static const uint8_t kRpcReplyQueue = 8;
  struct RpcReply {
    char topic[NbiotProtocol::kMqttTopicLen] = {0};
    char payload[512] = {0};  // la copia en el nodo lleva commandLog + un campo por umbral
  };
  RpcReply _rpcReplies[kRpcReplyQueue];
  uint8_t _rpcReplyHead = 0;   // indice de la siguiente que se manda
  uint8_t _rpcReplyCount = 0;

  bool rpcReplyPending() const { return _rpcReplyCount > 0; }
  void popRpcReply() {
    if (_rpcReplyCount == 0) return;
    _rpcReplyHead = (uint8_t)((_rpcReplyHead + 1) % kRpcReplyQueue);
    _rpcReplyCount--;
  }
  bool _rpcReplyInFlight = false;

  uint16_t _subMsgId = 1;  // id de mensaje de AT+QMTSUB, sube con cada suscripcion
  uint32_t _downlinksReceived = 0;

  // ---------- Anillo de telemetria pendiente de subir ----------
  SensorSnapshot _ring[NBIOT_RING_CAPACITY];
  uint8_t _ringHead = 0;
  uint8_t _ringCount = 0;
  uint32_t _oldestPendingMs = 0;

  // ---------- Lotes ----------
  // PUBLISHING se dispara cuando el anillo llega a _batchReadingsTarget lecturas O
  // pasan _batchSecondsTarget segundos desde la lectura pendiente mas antigua (o por
  // requestFlush()), y entonces vacia el anillo en publicaciones de
  // MQTT_PUB_CHUNK_READINGS lecturas. Los dos objetivos se pueden cambiar desde
  // Modulo A (setBatchSeconds / setBatchReadings).
  uint16_t _batchReadingsTarget = MQTT_BATCH_MAX_READINGS;
  uint32_t _batchSecondsTarget = MQTT_BATCH_SECONDS;

  // ---------- Observabilidad ----------
  bool _attached = false;
  uint32_t _attachedSinceMs = 0;
  int _rssiDbm = 0;
  uint32_t _packetsSent = 0;
  uint32_t _packetsFailed = 0;
  uint32_t _packetsDropped = 0;
};
