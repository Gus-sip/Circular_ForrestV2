#include "RYLR998.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// Ajusta la potencia de transmision (0..22 dBm). Ver la explicacion en RYLR998.h.
bool RYLR998::setTxPower(uint8_t dbm, Print *debug) {
  if (dbm > 22) dbm = 22;
  char cmd[24];
  snprintf(cmd, sizeof(cmd), "AT+CRFOP=%u", dbm);
  return sendATCommand(cmd, debug);
}

// Abre el UART y configura la radio paso a paso. Cada comando tiene que contestar
// +OK; al primer fallo se abandona y se devuelve false.
bool RYLR998::begin(uint16_t addr, uint16_t networkId, uint32_t bandHz, const RYLR998Params &params,
                     Print *debug) {
  _serial.begin(_baud, SERIAL_8N1, _rxPin, _txPin);

  char cmd[48];

  // Contesta el modulo? Si esto falla, es cableado, velocidad o alimentacion.
  if (!sendATCommand("AT", debug)) return false;

  // Direccion propia de esta radio. Tiene que ser distinta en cada nodo.
  snprintf(cmd, sizeof(cmd), "AT+ADDRESS=%u", addr);
  if (!sendATCommand(cmd, debug)) return false;

  // Red: solo se oyen radios con el mismo NETWORKID.
  snprintf(cmd, sizeof(cmd), "AT+NETWORKID=%u", networkId);
  if (!sendATCommand(cmd, debug)) return false;

  // Frecuencia en Hz (868 MHz en Europa).
  snprintf(cmd, sizeof(cmd), "AT+BAND=%lu", (unsigned long)bandHz);
  if (!sendATCommand(cmd, debug)) return false;

  // Parametros de modulacion. Emisor y receptor tienen que coincidir.
  snprintf(cmd, sizeof(cmd), "AT+PARAMETER=%u,%u,%u,%u", params.spreadingFactor, params.bandwidth,
           params.codingRate, params.preamble);
  if (!sendATCommand(cmd, debug)) return false;

  return true;
}

// Envia un paquete: cabecera "AT+SEND=<destino>,<longitud>," + los datos tal cual
// + fin de linea, y espera el +OK del modulo.
bool RYLR998::send(uint16_t destAddr, const char *data, uint8_t len) {
  char header[24];
  snprintf(header, sizeof(header), "AT+SEND=%u,%u,", destAddr, len);
  _serial.print(header);
  _serial.write(reinterpret_cast<const uint8_t *>(data), len);
  _serial.print("\r\n");
  return waitForOK(kSendTimeoutMs, nullptr);
}

// Manda un comando AT (opcionalmente mostrandolo por `debug`) y espera +OK.
bool RYLR998::sendATCommand(const char *cmd, Print *debug, uint32_t timeoutMs) {
  if (debug) {
    debug->print("  -> ");
    debug->println(cmd);
  }
  _serial.print(cmd);
  _serial.print("\r\n");
  bool ok = waitForOK(timeoutMs, debug);
  if (debug && !ok) debug->println("     (no +OK - check wiring/baud or that value is accepted)");
  return ok;
}

// Lee lineas del modulo hasta ver +OK (true), +ERR (false) o agotar el tiempo
// (false).
bool RYLR998::waitForOK(uint32_t timeoutMs, Print *debug) {
  uint32_t start = millis();
  char line[kLineMax + 1];
  uint16_t lineLen = 0;

  while (millis() - start < timeoutMs) {
    while (_serial.available()) {
      char c = _serial.read();

      if (c == '\n') {
        if (lineLen > 0) {
          line[lineLen] = '\0';
          if (debug) {
            debug->print("  <- ");
            debug->println(line);
          }
          if (strncmp(line, "+OK", 3) == 0) return true;
          if (strncmp(line, "+ERR", 4) == 0) return false;
          // Puede ser un +RCV= de un paquete que llego justo mientras se esperaba
          // la respuesta a este comando. Se guarda para poll() en vez de tragarse
          // en silencio un paquete real.
          if (!_hasPending && tryParseLine(line, lineLen, _pending)) _hasPending = true;
        }
        lineLen = 0;
      } else if (c != '\r') {
        if (lineLen < kLineMax) {
          line[lineLen++] = c;
        } else {
          lineLen = 0;  // linea demasiado larga: es basura, se descarta
        }
      }
    }
  }
  return false;
}

// No bloqueante. Primero entrega el paquete guardado por waitForOK(), si lo hay;
// despues procesa solo los bytes que ya esten en el buffer del UART.
bool RYLR998::poll(LoRaMessage &outMsg) {
  if (_hasPending) {
    outMsg = _pending;
    _hasPending = false;
    return true;
  }

  while (_serial.available()) {
    char c = _serial.read();

    if (c == '\n') {
      bool matched = false;
      if (_lineLen > 0) {
        _line[_lineLen] = '\0';
        matched = tryParseLine(_line, _lineLen, outMsg);
      }
      _lineLen = 0;
      if (matched) return true;
    } else if (c != '\r') {
      if (_lineLen < kLineMax) {
        _line[_lineLen++] = c;
      } else {
        _lineLen = 0;
      }
    }
  }
  return false;
}

// Interpreta "+RCV=<direccion>,<longitud>,<datos>,<rssi>,<snr>".
//
// Se guia por la LONGITUD declarada en vez de partir ingenuamente por comas, para
// que una coma dentro de los datos (las tramas de telemetria estan llenas de ellas)
// no descoloque los campos que vienen detras.
//
// Lo que NO resiste: un salto de linea dentro de los datos, porque poll() y
// waitForOK() cortan las lineas por '\n' antes de llegar aqui. Ni la telemetria de
// Modulo C ni los comandos CFG/ACK llevan ninguno, pero no esta protegido.
bool RYLR998::tryParseLine(char *line, uint16_t len, LoRaMessage &outMsg) {
  if (len < 7 || memcmp(line, "+RCV=", 5) != 0) return false;

  // Direccion del emisor.
  char *p = line + 5;
  char *comma1 = strchr(p, ',');
  if (!comma1) return false;
  *comma1 = '\0';
  int addr = atoi(p);

  // Longitud de los datos.
  char *lenStart = comma1 + 1;
  char *comma2 = strchr(lenStart, ',');
  if (!comma2) return false;
  *comma2 = '\0';
  int dataLen = atoi(lenStart);
  if (dataLen < 0 || dataLen > (int)(sizeof(outMsg.payload) - 1)) return false;

  // Los datos: exactamente dataLen bytes, seguidos de una coma.
  char *dataStart = comma2 + 1;
  if ((dataStart - line) + dataLen + 1 > len) return false;
  if (dataStart[dataLen] != ',') return false;

  // RSSI y SNR, despues de los datos.
  char *rssiStart = dataStart + dataLen + 1;
  char *comma3 = strchr(rssiStart, ',');
  if (!comma3) return false;
  *comma3 = '\0';
  int rssi = atoi(rssiStart);

  char *snrStart = comma3 + 1;
  int snr = atoi(snrStart);

  memcpy(outMsg.payload, dataStart, dataLen);
  outMsg.payload[dataLen] = '\0';
  outMsg.length = (uint8_t)dataLen;
  outMsg.senderAddr = (uint16_t)addr;
  outMsg.rssi = (int16_t)rssi;
  outMsg.snr = (int8_t)snr;
  return true;
}
