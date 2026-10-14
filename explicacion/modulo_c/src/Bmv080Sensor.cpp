#include "Bmv080Sensor.h"

// Espera tras un reinicio antes de que el sensor acepte una orden de medida.
#define BMV080_POST_RESET_MS 100
// Pausa entre intentos de arrancar el laser.
#define BMV080_START_RETRY_MS 150

// Mas pila para la tarea de loop: bmv080_serve_interrupt la necesita.
#if defined(ESP32)
SET_LOOP_TASK_STACK_SIZE(60 * 1024);
#endif

// ---- Funciones que el SDK de Bosch llama para hablar con el bus I2C ----

// Leer registros. En I2C la cabecera se desplaza un bit a la izquierda.
int8_t Bmv080Sensor::readCB(bmv080_sercom_handle_t handle, uint16_t header, uint16_t *payload,
                             uint16_t payloadLength) {
  if (handle == nullptr) return -5;
  sfTkIBus *bus = (sfTkIBus *)handle;
  if (bus->type() == ksfTkBusTypeI2C) header = header << 1;
  size_t nRead = 0;
  sfTkError_t rc = bus->readRegister(header, payload, payloadLength, nRead);
  return (rc == ksfTkErrOk && nRead == payloadLength) ? 0 : -2;
}

// Escribir registros.
int8_t Bmv080Sensor::writeCB(bmv080_sercom_handle_t handle, uint16_t header, const uint16_t *payload,
                              uint16_t payloadLength) {
  if (handle == nullptr) return -5;
  sfTkIBus *bus = (sfTkIBus *)handle;
  if (bus->type() == ksfTkBusTypeI2C) header = header << 1;
  sfTkError_t rc = bus->writeRegister(header, payload, payloadLength);
  return (rc == ksfTkErrOk) ? 0 : -3;
}

// Esperar el tiempo que pida el SDK.
int8_t Bmv080Sensor::delayCB(uint32_t durationMs) {
  delay(durationMs);
  return 0;
}

// El SDK llama aqui cuando tiene una medida nueva: se guarda para read().
void Bmv080Sensor::onDataReady(bmv080_output_t output, void *callbackParameters) {
  Bmv080Sensor *self = (Bmv080Sensor *)callbackParameters;
  self->_lastOutput = output;
  self->_dataReady = true;
}

// Abre el sensor probando todas las direcciones posibles, tres pasadas, y lo deja
// configurado pero SIN medir (laser apagado).
bool Bmv080Sensor::begin() {
  // La direccion I2C la fijan los pines CS/SDO, y de verdad cambia entre placas:
  // el nodo C1 contesta en 0x57 y el C2 en 0x56. En vez de exigir una compilacion
  // por placa - inviable en un despliegue de 200 nodos - se prueba primero la
  // direccion configurada y luego el resto del rango, igual que Bme690Sensor hace
  // con su pareja 0x76/0x77.
  //
  //   CS alto, SDO alto -> 0x57      CS bajo, SDO alto -> 0x55
  //   CS alto, SDO bajo -> 0x56      CS bajo, SDO bajo -> 0x54
  static const uint8_t kStrapAddrs[4] = {0x57, 0x56, 0x55, 0x54};

  bmv080_status_code_t rc = E_BMV080_ERROR_HW_WRITE;

  // TRES PASADAS POR EL RANGO, no una. El fallo es intermitente - la misma placa
  // abre en un arranque y no en el siguiente - asi que repetir es lo que mas
  // sube la probabilidad de exito por unidad de esfuerzo.
  for (uint8_t pasada = 0; pasada < kOpenPasses && rc != E_BMV080_OK; pasada++) {
    if (pasada > 0) {
      // Una pausa entre pasadas. El ciclo del pin de habilitacion seria mejor -
      // devolveria el chip a un estado conocido - pero ese pin lo conduce el
      // firmware principal, no este driver, y meterlo aqui le daria al driver una
      // dependencia de pins.h que no tiene.
      delay(kOpenRetryMs);
    }

    for (uint8_t attempt = 0; attempt < 5; attempt++) {
      // El intento 0 usa la direccion configurada; los siguientes recorren el
      // rango y se saltan la que ya se probo.
      uint8_t tryAddr = (attempt == 0) ? _addr : kStrapAddrs[attempt - 1];
      if (attempt > 0 && tryAddr == _addr) continue;

      _bus.init(_wire, tryAddr);
      _bus.setByteOrder(SFTK_MSBFIRST);

      rc = bmv080_open(&_handle, (bmv080_sercom_handle_t)&_bus, (bmv080_callback_read_t)readCB,
                       (bmv080_callback_write_t)writeCB, (bmv080_callback_delay_t)delayCB);
      if (rc == E_BMV080_OK) {
        _addr = tryAddr;  // recordarla, para que read() y cualquier reintento usen la buena
        _openPasses = (uint8_t)(pasada + 1);
        break;
      }
      // Si bmv080_open falla no deja nada abierto, asi que no hay que cerrar nada.
      _handle = nullptr;
    }
  }
  _lastOpenStatus = (int)rc;
  if (rc != E_BMV080_OK) {
    _failStage = FailStage::Open;
    return false;
  }

  bmv080_reset(_handle);
  // Tras un reinicio hay que esperar antes de que el sensor acepte una orden de
  // medida. Sin esta pausa, el primer bmv080_start_continuous_measurement() despues
  // de begin() falla.
  delay(BMV080_POST_RESET_MS);

  bool doObstructionDetection = true;
  bmv080_set_parameter(_handle, "do_obstruction_detection", (void *)&doObstructionDetection);

  // AQUI NO SE ARRANCA NINGUNA MEDIDA.
  //
  // Antes begin() terminaba con bmv080_start_continuous_measurement() y devolvia su
  // resultado, lo que juntaba dos fallos completamente distintos bajo un mismo "NOT
  // FOUND": el sensor ausente, y el sensor presente pero incapaz de ARRANCAR SU
  // LASER. El laser es la carga de 68 mA que ha tumbado esta placa repetidas veces,
  // asi que el segundo caso era el probable - y se informaba como sensor ausente,
  // lo que mandaba el diagnostico hacia el bus I2C y el hardware en vez de hacia
  // la alimentacion.
  //
  // Ademas era trabajo tirado: quien llama para la medida en el acto, porque el
  // laser solo se enciende dentro del turno del propio BMV080, con todo lo demas en
  // reposo. Abrir el sensor es el trabajo de begin(); medir es el del turno.
  //
  // Asi que begin() tiene exito cuando el sensor ABRE. Un laser que no arranca se
  // informa despues, en el turno, como fallo de lectura - que es lo que es.
  _measuring = false;
  _failStage = FailStage::None;
  return true;
}

// Enciende el laser y empieza a medir de forma continua.
bool Bmv080Sensor::startMeasurement() {
  if (_handle == nullptr) return false;
  if (_measuring) return true;

  // Se reintenta el arranque. Aqui es donde se enciende de verdad el laser - la
  // carga de 68 mA que ha tumbado esta placa varias veces - asi que una negativa no
  // prueba que el sensor este roto, y reintentar tras una pausa corta no cuesta nada
  // si va bien. Se guarda el estado del SDK para poder diagnosticar un fallo
  // persistente en vez de adivinarlo.
  bmv080_status_code_t rc = E_BMV080_ERROR_HW_WRITE;
  for (int attempt = 0; attempt < 3; attempt++) {
    rc = bmv080_start_continuous_measurement(_handle);
    if (rc == E_BMV080_OK) break;
    delay(BMV080_START_RETRY_MS);
  }

  _lastStartStatus = (int)rc;
  _measuring = (rc == E_BMV080_OK);
  _dataReady = false;
  return _measuring;
}

// Apaga el laser.
bool Bmv080Sensor::stopMeasurement() {
  if (_handle == nullptr || !_measuring) return true;
  bmv080_status_code_t rc = bmv080_stop_measurement(_handle);
  _measuring = false;  // se da por apagado pase lo que pase: nunca creer el laser encendido
  _dataReady = false;
  return rc == E_BMV080_OK;
}

// Atiende al SDK; si en esta llamada entrego una medida (onDataReady), la devuelve.
Reading Bmv080Sensor::read() {
  Reading r;
  _dataReady = false;
  bmv080_status_code_t rc = bmv080_serve_interrupt(_handle, onDataReady, this);

  if (rc != E_BMV080_OK) {
    r.status = (rc == E_BMV080_ERROR_HW_WRITE || rc == E_BMV080_ERROR_HW_READ) ? ReadingStatus::NoAck
                                                                                : ReadingStatus::InvalidFrame;
    return r;
  }

  if (!_dataReady) {
    r.status = ReadingStatus::NotReady;
    return r;
  }

  r.status = ReadingStatus::Ok;
  r.values[0] = _lastOutput.pm1_mass_concentration;
  r.values[1] = _lastOutput.pm2_5_mass_concentration;
  r.values[2] = _lastOutput.pm10_mass_concentration;
  r.values[3] = _lastOutput.is_obstructed ? 1.0f : 0.0f;
  r.count = 4;
  return r;
}

// Dormir = apagar el laser.
void Bmv080Sensor::sleep() {
  stopMeasurement();
}
