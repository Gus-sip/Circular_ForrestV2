#pragma once

#include <Arduino.h>

// =============================================================================
// DRIVER DE LA RADIO LoRa REYAX RYLR998
// =============================================================================
//
// Lo comparten Modulo C (transmite) y Modulo B (recibe). Antes habia una copia en
// cada modulo que solo se diferenciaba en los comentarios; se unifico aqui porque
// el driver era identico. Ver radio/README.md para los parametros de enlace de
// partida y como cambian alcance por tiempo en el aire.
//
// La radio se maneja con comandos de texto AT por un puerto serie (UART): se le
// manda "AT+SEND=..." y contesta "+OK" o "+ERR=..."; cuando llega un paquete por
// el aire, la radio escribe sola una linea "+RCV=...".

// Un paquete recibido, interpretado a partir de una linea
// "+RCV=<direccion>,<longitud>,<datos>,<rssi>,<snr>".
// El limite de 240 bytes de carga es el que documenta habitualmente AT+SEND del
// RYLR998. No se ha sacado de una hoja de datos incluida en este repositorio (no
// hay ninguna), asi que es una suposicion de partida a verificar contra el
// firmware real del modulo.
struct LoRaMessage {
  uint16_t senderAddr = 0;    // direccion AT+ADDRESS de quien lo envio
  char payload[241] = {0};    // los datos, terminados en '\0'
  uint8_t length = 0;         // longitud de los datos
  int16_t rssi = 0;           // intensidad de la senal recibida (dBm)
  int8_t snr = 0;             // relacion senal/ruido (dB)
};

// AT+PARAMETER=<factorDeEnsanchamiento>,<anchoDeBanda>,<tasaDeCodificacion>,<preambulo>.
// El ancho de banda es el INDICE/codigo del modulo, no un valor en kHz - ver
// radio/README.md para los valores de partida y su efecto en alcance y tiempo.
struct RYLR998Params {
  uint8_t spreadingFactor;  // SF: mas alto = mas alcance y mas tiempo en el aire
  uint8_t bandwidth;        // codigo de ancho de banda (7 = 125 kHz)
  uint8_t codingRate;       // tasa de codificacion (1 = 4/5)
  uint16_t preamble;        // longitud del preambulo
};

// Driver minimo por comandos AT.
//
// begin() y send() son BLOQUEANTES: esperan la respuesta rapida +OK/+ERR del
// modulo. poll() NO lo es: solo vacia los bytes que ya estan en el buffer de
// recepcion del UART, asi que se puede llamar en cada vuelta de loop() sin que
// la radio detenga nunca el programa (en Modulo B en particular, nunca debe
// detener el servidor web de handleClient()).
class RYLR998 {
public:
  RYLR998(HardwareSerial &serial, uint8_t rxPin, uint8_t txPin, uint32_t baud = 115200)
      : _serial(serial), _rxPin(rxPin), _txPin(txPin), _baud(baud) {}

  // Bloqueante: ejecuta AT, AT+ADDRESS, AT+NETWORKID, AT+BAND y AT+PARAMETER por
  // orden, y abandona en el primero que no conteste +OK. Si devuelve false, revisa
  // primero el cableado y la velocidad (rylr998_bridge.cpp) antes de suponer que
  // un parametro es incorrecto.
  // Pasando un flujo `debug` (por ejemplo &Serial) imprime cada comando y su
  // respuesta en el momento, para confirmar que el modulo acepto cada ajuste en
  // vez de fiarse de un unico verdadero/falso al final.
  bool begin(uint16_t addr, uint16_t networkId, uint32_t bandHz, const RYLR998Params &params,
             Print *debug = nullptr);

  // Bloqueante: escribe AT+SEND=<direccion>,<longitud>,<datos> y espera +OK, con
  // un limite de kSendTimeoutMs. El tiempo real en el aire depende de la longitud
  // y del SF/BW/CR configurados, pero esta llamada nunca espera indefinidamente.
  bool send(uint16_t destAddr, const char *data, uint8_t len);

  // Bloqueante: AT+CRFOP=<dbm>, de 0 a 22. El modulo arranca a 22 dBm, y su rafaga
  // de transmision pide ~120 mA - suficiente para tumbar una alimentacion que no
  // aguanta un escalon de carga, que es exactamente lo que le pasa a la placa de
  // Modulo C (se reiniciaba en cada transmision). Menos potencia cambia alcance
  // por una rafaga mas pequena. NO se incluye en begin() a proposito: este archivo
  // lo comparte Modulo B, que tiene alimentacion de sobra y ningun motivo para
  // renunciar a alcance.
  bool setTxPower(uint8_t dbm, Print *debug = nullptr);

  // No bloqueante: vacia lo que haya en el buffer y devuelve true (con outMsg
  // relleno) en cuanto encuentra una linea +RCV= completa y bien formada. Todo lo
  // demas que vea por el camino (ecos +OK sueltos, basura) se descarta.
  bool poll(LoRaMessage &outMsg);

private:
  static constexpr uint16_t kLineMax = 260;  // "+RCV=" + dir + long + 240 B de datos + rssi + snr
  static constexpr uint32_t kSendTimeoutMs = 2000;

  HardwareSerial &_serial;
  uint8_t _rxPin, _txPin;
  uint32_t _baud;

  // Linea en construccion para poll(): los bytes llegan sueltos y se juntan aqui
  // hasta ver el fin de linea.
  char _line[kLineMax + 1] = {0};
  uint16_t _lineLen = 0;

  // Un paquete que llego mientras se esperaba el +OK de un comando. Se guarda
  // aqui para que el siguiente poll() lo entregue en vez de perderlo.
  LoRaMessage _pending;
  bool _hasPending = false;

  bool sendATCommand(const char *cmd, Print *debug, uint32_t timeoutMs = 1000);
  bool waitForOK(uint32_t timeoutMs, Print *debug);
  static bool tryParseLine(char *line, uint16_t len, LoRaMessage &outMsg);
};
