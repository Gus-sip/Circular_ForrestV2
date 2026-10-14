#pragma once

// GESTOR DE ALIMENTACION - VERSION DE PRUEBA (stub), DE LA EPOCA DEL BANCO.
//
// Se escribio cuando el banco de pruebas aun no tenia transistores de corte (un
// unico rail de 3V3 siempre encendido), para que las dependencias de alimentacion
// quedaran expresadas en el codigo desde el principio.
//
// OJO: hoy los rails los controla directamente el firmware principal
// (enableRails() / enterDeepSleep() en chip_forest_lora_tx.cpp, con los pines de
// pins.h). Esta clase NO enciende ni apaga nada: solo apunta un estado y lo
// imprime por el puerto serie.
class PowerManager {
public:
  void enable3V3Sensors();
  void disable3V3Sensors();
  bool sensors3V3Enabled() const { return _sensors3V3; }

private:
  bool _sensors3V3 = false;
};
