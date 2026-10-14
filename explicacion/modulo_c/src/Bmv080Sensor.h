#pragma once

#include "ISensor.h"
#include <Wire.h>
#include <sfTkArdI2C.h>
#include <sfTk/bmv080.h>
#include <sfTk/bmv080_defs.h>

// =============================================================================
// BMV080 (Bosch) - PARTICULAS PM1 / PM2.5 / PM10
// =============================================================================
//
// Bus I2C compartido. Usa directamente el SDK de Bosch en vez del envoltorio de
// SparkFun para Arduino (SparkFunBMV080/sfDevBMV080): ese envoltorio reduce cada
// bmv080_status_code_t a un simple verdadero/falso, y eso hizo invisible el fallo
// de cableado original (bmv080_open fallando con E_BMV080_ERROR_HW_WRITE). Las
// funciones de lectura/escritura de abajo estan copiadas de la implementacion
// privada del propio envoltorio, no reinventadas: desplazar la cabecera de 16 bits
// un bit a la izquierda para I2C y delegar en sfTkIBus::readRegister/writeRegister,
// con el byte mas significativo primero.
//
// SET_LOOP_TASK_STACK_SIZE(60*1024) en el .cpp es obligatorio:
// bmv080_serve_interrupt necesita mas pila de la que el ESP32 da por defecto a la
// tarea de loop (segun el comentario de la propia libreria de SparkFun).
//
// Los ficheros del SDK de Bosch (bmv080.h, bmv080_defs.h, lib_bmv080.a,
// lib_postProcessor.a) NO vienen en la libreria publica: se copian a mano desde
// bosch_bmv080_sdk/ con restore.ps1 (ver platformio.ini).
//
// values[]: [0] = PM1, [1] = PM2.5, [2] = PM10 (ug/m3), [3] = obstruido (1/0).
// Necesita varios segundos funcionando antes de que una lectura sea valida
// (BMV080_STARTUP_DELAY_MS en Config.h cubre arranque, precalentamiento del laser y
// autotest). Pueden llegar lecturas antes de que el valor se estabilice del todo,
// porque is_outside_measurement_range es el unico indicador de validez que expone
// el SDK.
class Bmv080Sensor : public ISensor {
public:
  explicit Bmv080Sensor(uint8_t addr, TwoWire &wire = Wire) : _addr(addr), _wire(wire) {}

  bool begin() override;
  Reading read() override;
  void sleep() override;
  const char *name() const override { return "BMV080"; }

  // Encendido por turnos. Con el laser funcionando consume ~68 mA, con diferencia
  // la mayor carga del sistema, asi que solo se enciende durante su propia medida.
  // Arrancar y parar reutilizan el sensor ya abierto, asi que ninguna de las dos
  // vuelve a pagar el coste de abrir/reiniciar/autotest.
  bool startMeasurement();
  bool stopMeasurement();
  bool isMeasuring() const { return _measuring; }

  // Por que fallo begin(), y el codigo de estado del SDK detras. Sin esto, begin()
  // juntaria "no esta en el bus en ninguna direccion" y "esta, pero el laser no
  // arranca" en un solo verdadero/falso - dos averias con causas y arreglos
  // completamente distintos.
  enum class FailStage { None, Open };
  FailStage failStage() const { return _failStage; }
  int lastOpenStatus() const { return _lastOpenStatus; }
  // Estado del SDK en el ultimo intento de arrancar el laser.
  int lastStartStatus() const { return _lastStartStatus; }

  // En que pasada abrio. Sirve para saber si los reintentos hacen falta de verdad
  // o si el sensor abre siempre a la primera. 0 = no abrio.
  uint8_t openPasses() const { return _openPasses; }

private:
  // Cuantas veces se recorre el rango de direcciones antes de rendirse, y la pausa
  // entre pasadas. El fallo de apertura es INTERMITENTE - la misma placa abre en un
  // arranque y no en el siguiente - asi que repetir el recorrido entero es lo que
  // mas sube la probabilidad de exito por unidad de esfuerzo.
  static const uint8_t kOpenPasses = 3;
  static const uint16_t kOpenRetryMs = 100;

  uint8_t _addr;
  uint8_t _openPasses = 0;
  TwoWire &_wire;
  sfTkArdI2C _bus;
  bmv080_handle_t _handle = nullptr;
  FailStage _failStage = FailStage::None;
  int _lastOpenStatus = 0;
  int _lastStartStatus = 0;
  bool _measuring = false;

  // Lo rellena onDataReady() cuando el SDK entrega una medida.
  volatile bool _dataReady = false;
  bmv080_output_t _lastOutput{};

  static int8_t readCB(bmv080_sercom_handle_t handle, uint16_t header, uint16_t *payload, uint16_t payloadLength);
  static int8_t writeCB(bmv080_sercom_handle_t handle, uint16_t header, const uint16_t *payload,
                         uint16_t payloadLength);
  static int8_t delayCB(uint32_t durationMs);
  static void onDataReady(bmv080_output_t output, void *callbackParameters);
};
