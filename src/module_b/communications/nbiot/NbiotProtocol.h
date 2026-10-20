#pragma once

#include <stdint.h>
#include <stddef.h>

#include "Config.h"
#include "telemetry/SensorSnapshot.h"

// =============================================================================
// PROTOCOLO DEL MODEM NB-IoT: INTERPRETES DE RESPUESTAS AT Y DE JSON
// =============================================================================
//
// Todo este archivo esta, a proposito, libre de Arduino (sin Arduino.h ni
// HardwareSerial) para poder probarlo con una prueba unitaria nativa en el PC (ver
// test/test_nbiot_protocol) sin el compilador del ESP32. ModemNBIoTMqtt es lo unico
// que habla con el UART; a este modulo le pasa lineas de texto y recibe resultados
// ya interpretados.
//
// QUE SE USA HOY: los interpretes de CEREG, CSQ, CCLK, los de MQTT (QMT*) y las
// funciones de JSON. Las de socket UDP (QIOPEN, QIURC, QIRD), parseDownlink() y la
// trama binaria (encodeBatch*) son de la via antigua por UDP, ya retirada: solo las
// usa la prueba unitaria.

namespace NbiotProtocol {

// ---------- Interpretes de lineas de respuesta AT ----------
// Cada uno recibe UNA linea de respuesta ya separada y terminada en '\0' (sin
// CR/LF, tal como la entrega el lector byte a byte de ModemNBIoTMqtt) y devuelve
// false si la linea no encaja en absoluto, para que quien llama pruebe el
// siguiente interprete o la trate como un aviso no reconocido en vez de adivinar.

// Estado de registro en la red movil (respuesta a AT+CEREG?).
enum class CeregState : uint8_t {
  NOT_REGISTERED = 0,
  REGISTERED_HOME = 1,      // registrado en la red propia
  SEARCHING = 2,
  DENIED = 3,
  UNKNOWN = 4,
  REGISTERED_ROAMING = 5,   // registrado en itinerancia
};

// Registrado = red propia o itinerancia.
inline bool cieregIsAttached(CeregState s) {
  return s == CeregState::REGISTERED_HOME || s == CeregState::REGISTERED_ROAMING;
}

// "+CEREG: <n>,<stat>[,...]" (aviso espontaneo) o "+CEREG: <stat>" (respuesta a la
// consulta con <n>=0). Solo hace falta <stat>, asi que se aceptan las dos formas.
bool parseCereg(const char *line, CeregState &out);

// "+CSQ: <rssi>,<ber>". <rssi>=99 significa "desconocido/no detectable".
// Devuelve false si la linea no se entiende; rssiDbm solo vale si devuelve true.
bool parseCsq(const char *line, int &rssiDbm);

// "+CCLK: \"yy/MM/dd,hh:mm:ss[+/-zz]\"" (las comillas y la zona horaria final son
// opcionales). <zz> es un numero con signo de pasos de 15 minutos respecto a UTC.
// Convierte a milisegundos de epoca Unix (UTC). Devuelve false si la linea no se
// entiende o los campos estan fuera de rango; quien llama tiene que comprobar
// ademas el valor (un modem sin hora de red contesta con una fecha de relleno de
// hacia el ano 2000, que se interpreta bien pero no es una hora real).
// OJO: este modem (BC660K-GL) da la hora YA EN UTC; ver el .cpp.
bool parseCclk(const char *line, int64_t &epochMs);

// HISTORICO (via UDP): "+QIOPEN: <connectID>,<err>", el aviso que sigue al OK de
// AT+QIOPEN. err==0 significa que el socket se abrio.
// [ELIMINAR] via UDP retirada: parseQiopen, parseQiurcRecv, parseQirdHeader, parseDownlink y encodeBatch* solo los usa la prueba unitaria test_nbiot_protocol. Quitar con sus pruebas.
bool parseQiopen(const char *line, int &connectId, int &err);

// HISTORICO (via UDP): "+QIURC: \"recv\",<connectID>,<dataLen>" - avisa de que hay
// <dataLen> bytes esperando para leerlos con AT+QIRD=<connectID>,<dataLen>.
bool parseQiurcRecv(const char *line, int &connectId, int &dataLen);

// HISTORICO (via UDP): "+QIRD: <len>" - la cabecera de la respuesta de AT+QIRD,
// seguida de <len> bytes en crudo en la(s) linea(s) siguiente(s). Aqui solo se
// interpreta la cabecera.
bool parseQirdHeader(const char *line, int &len);

// ---------- Avisos MQTT (juego de comandos MQTT del BC660K-GL) ----------
// La sintaxis sigue la forma QMT estandar de Quectel, comun a las familias
// BC66/BC660K/EC. Si alguno vuelve sin entenderse, comprobarlo contra la "BC660K-GL
// MQTT Application Note" antes de suponer que la disposicion de campos esta mal.

// "+QMTOPEN: <client_idx>,<result>" - aviso tras el OK de AT+QMTOPEN.
// result==0 significa que se abrio la conexion TCP con el servidor.
bool parseQmtopen(const char *line, int &clientIdx, int &result);

// "+QMTCONN: <client_idx>,<result>,<retCode>" - aviso tras el OK de AT+QMTCONN.
// result==0 significa que el paquete CONNECT salio y hubo respuesta; retCode==0
// significa que el servidor acepto la conexion.
bool parseQmtconn(const char *line, int &clientIdx, int &result, int &retCode);

// "+QMTPUB: <client_idx>,<msgId>,<result>[,<value>]" - aviso tras AT+QMTPUB.
// result==0 significa publicacion correcta (value solo importa con QoS>0, que aqui
// no se usa: la telemetria se publica con QoS 0).
bool parseQmtpub(const char *line, int &clientIdx, int &msgId, int &result);

// "+QMTSTAT: <client_idx>,<errCode>" - espontaneo: el cliente MQTT se ha caido (el
// servidor o la red lo cerraron, se agoto el keepalive, etc.). No va ligado a ningun
// comando en curso: siempre entra por el camino de avisos.
bool parseQmtstat(const char *line, int &clientIdx, int &errCode);

// "+QMTSUB: <client_idx>,<msgID>,<result>[,<value>]" - aviso tras el OK de
// AT+QMTSUB. result==0 significa que la suscripcion se acepto.
bool parseQmtsub(const char *line, int &clientIdx, int &msgId, int &result);

// ---------- Mensaje MQTT entrante (ordenes desde Modulo A / ThingsBoard) ----------

constexpr size_t kMqttTopicLen = 72;
constexpr size_t kMqttPayloadLen = 320;

struct MqttMessage {
  int clientIdx = 0;
  int msgId = 0;
  char topic[kMqttTopicLen] = {0};
  char payload[kMqttPayloadLen] = {0};
};

// "+QMTRECV: <client_idx>,<msgID>,"<topic>"[,<payload_len>],"<payload>""
// Se aceptan las dos formas (con y sin la longitud explicita): cual emite el modulo
// depende de AT+QMTCFG="recv/mode". El contenido se toma del ultimo tramo entre
// comillas de la linea, para que un contenido con comas (todo JSON las tiene) no
// descoloque los campos de detras.
bool parseQmtrecv(const char *line, MqttMessage &out);

// ---------- Extraccion minima de campos JSON ----------
// Lo justo para los pequenos mensajes RPC de ThingsBoard: sin memoria dinamica, sin
// libreria, sin analizar el JSON entero. Cada funcion busca "<clave>" y lee el
// valor que la sigue, lo que no es ambiguo en estos mensajes porque sus claves son
// distintas y anidan como mucho dos niveles. NO reutilizarlo para JSON arbitrario:
// no entiende de ambitos, asi que una clave repetida a otra profundidad
// coincidiria con la primera aparicion.
bool jsonString(const char *json, const char *key, char *out, size_t cap);

// Lee el valor de `key` sea cadena entrecomillada O NUMERO JSON DESNUDO, y lo
// devuelve como texto tal cual venia.
//
// jsonString() exige comillas, y la v2 del protocolo A->C dice explicitamente:
// "Numeros con decimal: van como numero JSON (1.5), no como texto". Los tres
// comandos NxT mandan su N asi, de modo que sin esto no se pueden leer.
//
// Se devuelve como TEXTO y no como float a proposito: el valor viaja despues
// dentro de una cadena CFG y es el nodo quien lo interpreta, asi que convertirlo
// aqui solo anadiria una conversion de ida y vuelta donde se pierde precision.
bool jsonScalar(const char *json, const char *key, char *out, size_t cap);
bool jsonInt(const char *json, const char *key, long &out);
// Extrae el objeto con llaves equilibradas que sigue a "<clave>": - se usa para
// sacar los subobjetos "data" y "params" de un RPC de pasarela.
bool jsonObject(const char *json, const char *key, char *out, size_t cap);

// Recorre las parejas clave/valor del cuerpo de un objeto JSON plano. Se llama con
// el cursor apuntando al objeto (o justo despues de su '{'); devuelve false cuando
// no quedan parejas. Los valores se devuelven como texto sin las comillas, asi que
// tanto {"A":1} como {"A":"1"} dan "1".
bool jsonNextPair(const char *&cursor, char *key, size_t keyCap, char *value, size_t valueCap);

// ---------- HISTORICO: gramatica de respuesta de la via UDP ----------
// El servidor contestaba a un datagrama de subida con:
//   ACK,<seq>
//   ACK,<seq>,CFG,<version>,<clave>=<valor>,<clave>=<valor>,...
// Texto plano. Aqui solo se extrae la estructura; que claves son validas y en que
// rango lo decidia ModemNBIoT con las constantes NBIOT_CFG_*_MIN/MAX de Config.h.
// Ya no lo usa la via MQTT.
constexpr uint8_t kMaxDownlinkKv = 4;
constexpr uint8_t kDownlinkKeyLen = 8;
constexpr uint8_t kDownlinkValueLen = 16;

struct DownlinkKv {
  char key[kDownlinkKeyLen] = {0};
  char value[kDownlinkValueLen] = {0};
};

struct DownlinkAck {
  bool valid = false;
  uint16_t seq = 0;
  bool hasCfg = false;
  uint8_t cfgVersion = 0;
  DownlinkKv kv[kMaxDownlinkKv];
  uint8_t kvCount = 0;
};

// text es el contenido UDP en crudo (ya terminado en '\0' por quien llama).
// Devuelve false solo si ni siquiera empieza por "ACK,<numero>"; lo que no encaje en
// la gramatica CFG se deja sin interpretar (out.hasCfg sigue a false) en vez de
// rechazar el ACK entero.
bool parseDownlink(const char *text, DownlinkAck &out);

// ---------- HISTORICO: trama binaria de lote de la via UDP ----------
// Un CSV de texto con una lectura completa de 13 campos mas hora y calidad de enlace
// ocupa ~75-80 bytes por lectura, y un lote de 10 superaba el presupuesto de 500
// bytes. Con campos binarios de coma fija una lectura baja a kBatchRecordSize bytes.
// (La via MQTT actual publica JSON y no usa esto.)
//
// Todos los campos de varios bytes van en big-endian ("orden de red"), escritos a
// mano byte a byte en vez de copiar una estructura con memcpy: sin depender del
// relleno del compilador ni de que el ESP32 y el servidor coincidan en orden por
// casualidad.
//
// Disposicion:
//   Cabecera (kBatchHeaderSize = 5 bytes):
//     [0]   magic        0xF0
//     [1:2] seq          uint16 BE, sube una vez por lote (da la vuelta)
//     [3]   count        uint8, numero de registros que siguen
//     [4]   cfgVersion   uint8, version del ultimo CFG aplicado (0 = ninguno)
//   Registro (kBatchRecordSize = 30 bytes), repetido `count` veces:
//     [0:1]   dtSec        uint16 BE, segundos desde la lectura MAS ANTIGUA del lote
//     [2:3]   rssi         int16 BE, RSSI LoRa (dBm) al recibir
//     [4]     snr          int8,  SNR LoRa
//     [5:6]   temp_x10     int16 BE, grados C * 10
//     [7:8]   hum_x10      uint16 BE, %HR * 10
//     [9:10]  pres_x10     uint16 BE, hPa * 10
//     [11:12] gas_x100ohm  uint16 BE, resistencia en unidades de 100 ohmios
//     [13:14] pm1_x10      uint16 BE, ug/m3 * 10
//     [15:16] pm25_x10     uint16 BE, ug/m3 * 10
//     [17:18] pm10_x10     uint16 BE, ug/m3 * 10
//     [19:20] co2_ppm      uint16 BE
//     [21:22] co_x100      uint16 BE, ppm * 100
//     [23:24] coTemp_x10   int16 BE, grados C * 10
//     [25:26] windAngle_x10 uint16 BE, grados * 10 (0-3599)
//     [27:28] windSpeed_x10 uint16 BE
//     [29]    flags        uint8, bit0 = windValid, bits 1-7 reservados (0)
constexpr uint8_t kBatchMagic = 0xF0;
constexpr size_t kBatchHeaderSize = 5;
constexpr size_t kBatchRecordSize = 30;

static_assert(kBatchHeaderSize + (size_t)NBIOT_CFG_READINGS_MAX * kBatchRecordSize <=
                  NBIOT_PAYLOAD_MAX_BYTES,
              "NBIOT_CFG_READINGS_MAX * kBatchRecordSize + header must fit under "
              "NBIOT_PAYLOAD_MAX_BYTES - lower NBIOT_CFG_READINGS_MAX or shrink the record");

// Escribe kBatchHeaderSize bytes en out (buffer de quien llama, con sitio suficiente).
void encodeBatchHeader(uint16_t seq, uint8_t count, uint8_t cfgVersion, uint8_t *out);

// Escribe kBatchRecordSize bytes en out (buffer de quien llama, con sitio suficiente).
void encodeBatchRecord(const SensorSnapshot &snap, uint16_t dtSec, uint8_t *out);

}  // namespace NbiotProtocol
