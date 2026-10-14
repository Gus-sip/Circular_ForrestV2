#include "Cm1106Sensor.h"

// Comando "leer CO2" del protocolo Cubic: 11 01 01 ED.
static const uint8_t READ_CO2_CMD[] = {0x11, 0x01, 0x01, 0xED};

// Enciende el sensor (EN alto), abre el UART y espera el calentamiento.
bool Cm1106Sensor::begin() {
  if (_enPin != kNoEnPin) {
    pinMode(_enPin, OUTPUT);
    digitalWrite(_enPin, HIGH);
  }
  _serial.begin(_baud, SERIAL_8N1, _rxPin, _txPin);
  if (_warmupMs > 0) delay(_warmupMs);
  return true;  // sin saludo inicial: la presencia se confirma con la primera lectura valida
}

// Manda el comando de lectura y espera hasta 1 s los 8 bytes de respuesta:
//   16 05 01 <CO2 alto> <CO2 bajo> <estado> <contador> <checksum>
Reading Cm1106Sensor::read() {
  Reading r;

  while (_serial.available()) _serial.read();
  _serial.write(READ_CO2_CMD, sizeof(READ_CO2_CMD));

  uint8_t resp[8];
  uint8_t len = 0;
  uint32_t start = millis();
  while (millis() - start < 1000 && len < sizeof(resp)) {
    if (_serial.available()) resp[len++] = _serial.read();
  }

  // Guardar los bytes del cable ANTES de decidir nada, para que quien llama pueda
  // distinguir una trama congelada pero nueva de un valor reciclado, acabe como
  // acabe esta lectura.
  memcpy(_prevRaw, _lastRaw, sizeof(_prevRaw));
  _prevRawLen = _lastRawLen;
  memcpy(_lastRaw, resp, sizeof(_lastRaw));
  _lastRawLen = len;

  if (len < 8) {
    r.status = ReadingStatus::Timeout;
    return r;
  }
  if (resp[0] != 0x16 || resp[1] != 0x05 || resp[2] != 0x01) {
    r.status = ReadingStatus::InvalidFrame;
    return r;
  }

  uint16_t sum = 0;
  for (uint8_t i = 0; i < 7; i++) sum += resp[i];
  uint8_t checksum = (uint8_t)(256 - (sum % 256));
  if (checksum != resp[7]) {
    r.status = ReadingStatus::InvalidFrame;
    return r;
  }

  r.status = ReadingStatus::Ok;
  const uint16_t value = (uint16_t)resp[3] * 256 + resp[4];

  // Vigilar si la MEDIDA se ha quedado clavada, no la trama - ver frozenValueRun().
  if (_haveValue && value == _lastValue) {
    if (_frozenRun < 0xFFFF) _frozenRun++;
  } else {
    _frozenRun = 0;
  }
  _lastValue = value;
  _haveValue = true;

  r.values[0] = value;
  r.count = 1;
  return r;
}

// Reposo = quitarle la alimentacion con EN.
void Cm1106Sensor::sleep() {
  if (_enPin != kNoEnPin) digitalWrite(_enPin, LOW);
}

void Cm1106Sensor::powerCycle(uint32_t offMs) {
  if (_enPin == kNoEnPin) return;
  digitalWrite(_enPin, LOW);
  delay(offMs);
  digitalWrite(_enPin, HIGH);
  if (_warmupMs > 0) delay(_warmupMs);
  // Las tramas de antes del ciclo no dicen nada del sensor despues de el.
  _frozenRun = 0;
  _lastRawLen = 0;
  _prevRawLen = 0;
  _haveValue = false;
  while (_serial.available()) _serial.read();
}
