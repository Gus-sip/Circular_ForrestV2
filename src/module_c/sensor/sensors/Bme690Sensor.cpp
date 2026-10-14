#include "Bme690Sensor.h"

#include <Wire.h>

// Lectura cruda del registro de identificacion del chip, saltandose por completo
// el driver de Bosch.
//
// Hace falta porque el begin() de la libreria resume varios fallos muy distintos
// en un solo `false`: la prueba de ACK en I2C, el reinicio por software de
// bme69x_init(), la comprobacion del ID, la lectura de variante y la carga de la
// calibracion. El nodo C2 mostro `ACK at 0x76` en el escaneo del bus y
// `BME690: NOT FOUND` en begin() en EL MISMO arranque: el aparato contesta a su
// direccion pero algo posterior falla, y sin el valor crudo no hay forma de
// distinguir una pieza equivocada de una lectura corrupta.
//
// El valor esperado es 0x61 (comun a BME680/688/690). En los foros hay casos de
// este registro devolviendo 0x00 o 0xF1 en buses que si dan ACK: es corrupcion del
// bus, no una pieza distinta. La prueba de direccion es un solo byte y sobrevive;
// la lectura de un registro es una transaccion mas larga y no.
//
// Dos formas de acceso, porque fallan por motivos distintos y un aparato que
// rechaza una puede contestar a la otra:
//
//   modo 0  escribir registro, INICIO REPETIDO, leer   - lo que hace el driver de Bosch
//   modo 1  escribir registro, STOP, y una lectura nueva aparte
//
// Si el modo 1 funciona donde falla el 0, la pieza esta viva y el problema es el
// inicio repetido; si fallan los dos pero la direccion sigue dando ACK, el aparato
// tiene corriente para tirar de la linea pero no para funcionar.
static bool readChipIdMode(uint8_t addr, uint8_t &out, int mode) {
  Wire.beginTransmission(addr);
  Wire.write(0xD0);  // BME69X_REG_CHIP_ID
  if (Wire.endTransmission(mode == 1) != 0) return false;
  if (mode == 1) delay(2);
  if (Wire.requestFrom((int)addr, 1) != 1) return false;
  out = (uint8_t)Wire.read();
  return true;
}

// Prueba los dos modos por orden y dice cual contesto (-1 si ninguno).
static bool readChipId(uint8_t addr, uint8_t &out, int &modeUsed) {
  for (int mode = 0; mode < 2; mode++) {
    if (readChipIdMode(addr, out, mode)) {
      modeUsed = mode;
      return true;
    }
    delay(5);
  }
  modeUsed = -1;
  return false;
}

// LA PRIMERA MEDIDA FORZADA DESPUES DE begin() SIEMPRE FALLA EN ESTA PIEZA.
//
// Medido en el nodo C2: lectura 1 -> Timeout, lectura 2 -> OK (gas 41451),
// lectura 3 -> OK, en todos los arranques. getData() de la libreria dispara una
// medida, espera la duracion calculada y trata cualquier estado distinto de OK de
// bme69x_get_data() como fallo - y los avisos de Bosch como NO_NEW_DATA son
// codigos POSITIVOS, asi que un aviso en la primera medida no se distingue de un
// error. El calefactor de gas tambien necesita una pasada antes de estabilizarse,
// y por eso es la primera medida la que avisa.
//
// Mientras el nodo leia sin parar, esto costaba una lectura de cada cien y paso
// desapercibido. Con el ciclo de sueno es fatal: cada despertar es un begin()
// nuevo seguido de UNA sola lectura, asi que la unica lectura que se hace es la que
// siempre falla, y el BME690 no aportaria nunca un valor. Por eso read() y
// readFast() reintentan una vez.

// Arranca el sensor, probando las dos direcciones y hasta tres veces.
bool Bme690Sensor::begin() {
  _chipId = 0;
  _chipIdValid = false;

  // Se repite el arranque entero en vez de rendirse al primer intento malo.
  //
  // bme69x_init() hace un REINICIO POR SOFTWARE y lee el ID justo despues. Una pieza
  // a la que se le acaba de encender el rail - que es cada despertar con el ciclo de
  // sueno - puede estar todavia estabilizandose cuando llega esa lectura, y el
  // driver devuelve el mismo `false` que para un sensor ausente de verdad. Tres
  // intentos con pausa cuestan ~150 ms si falla y nada en absoluto si va bien.
  for (int attempt = 0; attempt < 3; attempt++) {
    if (_sensor.begin(_addr)) {
      primeAndMark();
      return true;
    }

    uint8_t altAddr = (_addr == 0x76) ? 0x77 : 0x76;
    if (_sensor.begin(altAddr)) {
      _addr = altAddr;
      primeAndMark();
      return true;
    }

    delay(50);
  }

  // Ha fallado. Se lee el ID crudo para que quien llama pueda decir POR QUE y no
  // solo "NOT FOUND": 0x61 = la pieza esta bien y falla el driver; 0x00 o basura =
  // el bus corrompe las lecturas; ninguna respuesta = no esta de verdad.
  _chipIdValid = readChipId(_addr, _chipId, _chipIdMode);
  return false;
}

// Marca el sensor como arrancado. A PROPOSITO NO HACE NINGUNA MEDIDA.
//
// Antes lanzaba aqui una medida de cebado que se descartaba, lo que significaba
// encender el calefactor a 320 C dentro de setup(), justo antes de bmv080.begin().
// El efecto se midio: con el cebado aqui el BME690 arrancaba y el BMV080 daba
// NoAck y el CM1106 Timeout; sin el, arrancaban el BMV080 y el CM1106 y el BME690
// no. Los sensores se turnaban.
//
// setup() arranca todos los sensores seguidos, asi que era el unico sitio donde no
// se aplicaba la regla de "un sensor cada vez" (los turnos gobiernan solo las
// LECTURAS). Un calefactor encendido ahi es justo la carga simultanea que los
// turnos existen para evitar.
//
// Ademas el cebado no hace falta: read() ya reintenta, lo que cubre el "la primera
// medida siempre falla", y lo hace dentro del turno del propio BME690, donde el
// calefactor tiene la alimentacion para el solo.
void Bme690Sensor::primeAndMark() {
  _primed = true;
  _chipId = 0x61;  // si arranco, el ID coincidia por definicion
  _chipIdValid = true;
  _chipIdMode = 0;
}

// Dormir = calefactor apagado. El calefactor es todo el consumo de este sensor; el
// resto del chip consume microamperios. Se llama antes de cortar el rail de 3V3,
// para dejar la pieza en reposo en vez de quitarle la corriente a media medida.
// Cortar el rail ahorra mucho mas (un sensor sin alimentacion no consume nada), asi
// que lo que aporta esto es un apagado ordenado, no el ahorro.
void Bme690Sensor::sleep() {
  setHeater(false);
}

// El calefactor es todo el coste de una lectura del BME690. Cambiarlo es una
// escritura de configuracion por I2C, asi que se recuerda el estado y solo se
// escribe cuando cambia de verdad.
//
// 320 C / 150 ms son los valores por defecto de la propia libreria, y se restauran
// al reactivarlo para que las cifras de gas sigan siendo comparables con las
// tomadas antes de que se tocara nunca el calefactor.
bool Bme690Sensor::setHeater(bool on) {
  if (_heaterOn == on) return true;
  if (!_sensor.setHeater(on, 320, 150)) return false;
  _heaterOn = on;
  return true;
}

// Temperatura, humedad y presion con el calefactor APAGADO - ver la cabecera.
Reading Bme690Sensor::readFast() {
  Reading r;

  if (!setHeater(false)) {
    r.status = ReadingStatus::NotReady;
    return r;
  }

  float t, h, pr, g;
  for (int attempt = 0; attempt < 2; attempt++) {
    if (_sensor.getData(t, h, pr, g)) {
      r.status = ReadingStatus::Ok;
      r.values[0] = t;
      r.values[1] = h;
      r.values[2] = pr;
      r.values[3] = 0.0f;  // no se midio gas: no hacerlo pasar por real
      r.count = 3;
      return r;
    }
    delay(20);
  }

  r.status = ReadingStatus::Timeout;
  return r;
}

// Lectura completa, con el calefactor encendido (readFast() puede haberlo apagado).
Reading Bme690Sensor::read() {
  setHeater(true);

  Reading r;
  float temperature, humidity, pressure, gas;

  // Un reintento. Cubre la primera medida que siempre falla y una medida que avise
  // a mitad, que de otro modo tiraria los datos ambientales de todo un despertar.
  // Dos fallos seguidos son una averia real y se informan.
  for (int attempt = 0; attempt < 2; attempt++) {
    if (_sensor.getData(temperature, humidity, pressure, gas)) {
      r.status = ReadingStatus::Ok;
      r.values[0] = temperature;
      r.values[1] = humidity;
      r.values[2] = pressure;
      r.values[3] = gas;
      r.count = 4;
      return r;
    }
    delay(20);
  }

  r.status = ReadingStatus::Timeout;
  return r;
}
