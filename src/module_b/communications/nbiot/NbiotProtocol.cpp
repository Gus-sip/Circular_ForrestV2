#include "NbiotProtocol.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

namespace NbiotProtocol {

namespace {

// Devuelve un puntero justo despues de la coma numero `count` contando desde p, o
// nullptr si no hay tantas. No modifica el texto: strtol() se para sola en el
// siguiente caracter no numerico, asi que nunca hace falta una subcadena terminada
// en '\0' para leer un campo numerico.
const char *skipCommas(const char *p, int count) {
  for (int i = 0; i < count; i++) {
    p = strchr(p, ',');
    if (!p) return nullptr;
    p++;
  }
  return p;
}

void putU16BE(uint8_t *out, uint16_t v) {
  out[0] = (uint8_t)(v >> 8);
  out[1] = (uint8_t)(v & 0xFF);
}

int16_t clampToInt16(long v) {
  if (v > 32767) return 32767;
  if (v < -32768) return -32768;
  return (int16_t)v;
}

uint16_t clampToUint16(long v) {
  if (v < 0) return 0;
  if (v > 65535) return 65535;
  return (uint16_t)v;
}

}  // namespace

bool parseCereg(const char *line, CeregState &out) {
  if (strncmp(line, "+CEREG:", 7) != 0) return false;
  const char *p = line + 7;
  while (*p == ' ') p++;
  // Solo se trata la forma de respuesta a la consulta "AT+CEREG?":
  // "+CEREG: <n>,<stat>[,tac,ci,act]". El modem siempre pregunta en vez de activar
  // los avisos espontaneos +CEREG, asi que es la unica forma que necesita leer. (La
  // forma espontanea es ambigua sin ese contexto: con +CEREG=2 su primer campo ES
  // <stat>, no <n> - una ambiguedad que las consultas no tienen.)
  const char *statField = skipCommas(p, 1);
  if (!statField) return false;
  long stat = strtol(statField, nullptr, 10);
  if (stat < 0 || stat > 5) return false;
  out = static_cast<CeregState>(stat);
  return true;
}

bool parseCsq(const char *line, int &rssiDbm) {
  if (strncmp(line, "+CSQ:", 5) != 0) return false;
  const char *p = line + 5;
  while (*p == ' ') p++;
  long raw = strtol(p, nullptr, 10);
  if (raw < 0 || raw > 31) return false;  // 99 (y cualquier otro fuera de rango) = desconocido
  rssiDbm = (int)(-113 + raw * 2);
  return true;
}

bool parseCclk(const char *line, int64_t &epochMs) {
  if (strncmp(line, "+CCLK:", 6) != 0) return false;
  const char *p = line + 6;
  while (*p == ' ' || *p == '"') p++;

  // yy/MM/dd,hh:mm:ss[+/-zz]
  char *end = nullptr;
  long yy = strtol(p, &end, 10);
  if (end == p || *end != '/') return false;
  p = end + 1;
  long mon = strtol(p, &end, 10);
  if (end == p || *end != '/') return false;
  p = end + 1;
  long day = strtol(p, &end, 10);
  if (end == p || *end != ',') return false;
  p = end + 1;
  long hh = strtol(p, &end, 10);
  if (end == p || *end != ':') return false;
  p = end + 1;
  long mm = strtol(p, &end, 10);
  if (end == p || *end != ':') return false;
  p = end + 1;
  long ss = strtol(p, &end, 10);
  if (end == p) return false;
  p = end;

  long tzQuarters = 0;
  if (*p == '+' || *p == '-') {
    int sign = (*p == '-') ? -1 : 1;
    p++;
    const char *q = p;
    long v = strtol(q, &end, 10);
    if (end != q) tzQuarters = sign * v;
  }

  if (mon < 1 || mon > 12 || day < 1 || day > 31 || hh < 0 || hh > 23 || mm < 0 || mm > 59 || ss < 0 ||
      ss > 60)
    return false;

  // days_from_civil (Howard Hinnant): dias desde el 1970-01-01, calendario
  // gregoriano, sin depender de ninguna libreria de fechas ni de zonas horarias.
  long y = 2000 + yy;
  unsigned m = (unsigned)mon;
  unsigned d = (unsigned)day;
  y -= (m <= 2);
  long era = (y >= 0 ? y : y - 399) / 400;
  unsigned yoe = (unsigned)(y - era * 400);
  unsigned doy = (153u * (m + (m > 2 ? -3 : 9)) + 2u) / 5u + d - 1u;
  unsigned doe = yoe * 365u + yoe / 4u - yoe / 100u + doy;
  long long days = (long long)era * 146097 + (long long)doe - 719468;

  long long secs = days * 86400LL + hh * 3600LL + mm * 60LL + ss;

  // LA HORA QUE DA ESTE MODULO YA ESTA EN UTC: no restar la zona horaria.
  //
  // La norma 3GPP dice que +CCLK? devuelve la hora LOCAL con la zona al lado, asi que
  // lo obvio seria "restar la zona para tener UTC". Este BC660K-GL no se comporta
  // asi: da UTC en el campo de la hora aunque siga anunciando la zona local. Restarla
  // corrige dos veces y pone todas las marcas de tiempo una zona entera en el pasado.
  //
  // Medido en el hardware el 2026-09-14:
  //   modem          +CCLK: 26/09/14,08:46:08+08   (+08 cuartos de hora = +2 h, horario de verano)
  //   UTC del PC     2026-09-14 08:45:54
  //   antes salia -> 2026-09-14 06:46:08    = 2,00 horas ANTES
  //
  // El campo de la hora coincide con UTC con los 14 s de ida y vuelta, asi que es UTC.
  //
  // tzQuarters se sigue leyendo - hay que consumirlo para validar la linea y sirve
  // para diagnosticar - pero a proposito NO se aplica. Si alguna vez se usa otro
  // modulo, comprobarlo contra un reloj conocido antes de volver a restar; el fallo
  // es silencioso y parece un problema de visualizacion del servidor, no de este
  // codigo.
  (void)tzQuarters;

  epochMs = (int64_t)secs * 1000LL;
  return true;
}

bool parseQiopen(const char *line, int &connectId, int &err) {
  if (strncmp(line, "+QIOPEN:", 8) != 0) return false;
  const char *p = line + 8;
  while (*p == ' ') p++;
  const char *errField = skipCommas(p, 1);
  if (!errField) return false;
  connectId = (int)strtol(p, nullptr, 10);
  err = (int)strtol(errField, nullptr, 10);
  return true;
}

bool parseQiurcRecv(const char *line, int &connectId, int &dataLen) {
  if (strncmp(line, "+QIURC:", 7) != 0) return false;
  const char *quoted = strstr(line, "\"recv\"");
  if (!quoted) return false;
  const char *idField = skipCommas(quoted + 6, 1);
  if (!idField) return false;
  const char *lenField = skipCommas(idField, 1);
  if (!lenField) return false;
  connectId = (int)strtol(idField, nullptr, 10);
  dataLen = (int)strtol(lenField, nullptr, 10);
  return true;
}

bool parseQirdHeader(const char *line, int &len) {
  if (strncmp(line, "+QIRD:", 6) != 0) return false;
  const char *p = line + 6;
  while (*p == ' ') p++;
  len = (int)strtol(p, nullptr, 10);
  return true;
}

bool parseQmtopen(const char *line, int &clientIdx, int &result) {
  if (strncmp(line, "+QMTOPEN:", 9) != 0) return false;
  const char *p = line + 9;
  while (*p == ' ') p++;
  const char *resultField = skipCommas(p, 1);
  if (!resultField) return false;
  clientIdx = (int)strtol(p, nullptr, 10);
  result = (int)strtol(resultField, nullptr, 10);
  return true;
}

bool parseQmtconn(const char *line, int &clientIdx, int &result, int &retCode) {
  if (strncmp(line, "+QMTCONN:", 9) != 0) return false;
  const char *p = line + 9;
  while (*p == ' ') p++;
  const char *resultField = skipCommas(p, 1);
  if (!resultField) return false;
  const char *retCodeField = skipCommas(p, 2);
  clientIdx = (int)strtol(p, nullptr, 10);
  result = (int)strtol(resultField, nullptr, 10);
  // retCode solo acompana a result==0 (llego una respuesta de verdad); en otro caso
  // falta, asi que no se rechaza la linea entera por un campo ausente.
  retCode = retCodeField ? (int)strtol(retCodeField, nullptr, 10) : -1;
  return true;
}

bool parseQmtpub(const char *line, int &clientIdx, int &msgId, int &result) {
  if (strncmp(line, "+QMTPUB:", 8) != 0) return false;
  const char *p = line + 8;
  while (*p == ' ') p++;
  const char *msgIdField = skipCommas(p, 1);
  if (!msgIdField) return false;
  const char *resultField = skipCommas(p, 2);
  if (!resultField) return false;
  clientIdx = (int)strtol(p, nullptr, 10);
  msgId = (int)strtol(msgIdField, nullptr, 10);
  result = (int)strtol(resultField, nullptr, 10);
  return true;
}

bool parseQmtstat(const char *line, int &clientIdx, int &errCode) {
  if (strncmp(line, "+QMTSTAT:", 9) != 0) return false;
  const char *p = line + 9;
  while (*p == ' ') p++;
  const char *errField = skipCommas(p, 1);
  if (!errField) return false;
  clientIdx = (int)strtol(p, nullptr, 10);
  errCode = (int)strtol(errField, nullptr, 10);
  return true;
}

bool parseQmtsub(const char *line, int &clientIdx, int &msgId, int &result) {
  if (strncmp(line, "+QMTSUB:", 8) != 0) return false;
  const char *p = line + 8;
  while (*p == ' ') p++;
  const char *msgIdField = skipCommas(p, 1);
  if (!msgIdField) return false;
  const char *resultField = skipCommas(p, 2);
  if (!resultField) return false;
  clientIdx = (int)strtol(p, nullptr, 10);
  msgId = (int)strtol(msgIdField, nullptr, 10);
  result = (int)strtol(resultField, nullptr, 10);
  return true;
}

bool parseQmtrecv(const char *line, MqttMessage &out) {
  if (strncmp(line, "+QMTRECV:", 9) != 0) return false;
  const char *p = line + 9;
  while (*p == ' ') p++;

  char *end = nullptr;
  long idx = strtol(p, &end, 10);
  if (end == p || *end != ',') return false;
  p = end + 1;
  long msg = strtol(p, &end, 10);
  if (end == p || *end != ',') return false;
  p = end + 1;

  // topic: el primer tramo entre comillas
  if (*p != '"') return false;
  const char *topicStart = p + 1;
  const char *topicEnd = strchr(topicStart, '"');
  if (!topicEnd) return false;
  size_t topicLen = (size_t)(topicEnd - topicStart);
  if (topicLen >= kMqttTopicLen) topicLen = kMqttTopicLen - 1;

  // contenido: desde la comilla que se abre tras el topic hasta la ULTIMA comilla de
  // la linea. Saltarse lo que haya en medio (un campo de longitud opcional) hace que
  // funcionen las dos formas del aviso, y tomar la ultima comilla impide que las
  // comas del JSON lo corten.
  const char *afterTopic = topicEnd + 1;
  const char *payStart = strchr(afterTopic, '"');
  if (!payStart) return false;
  payStart++;
  const char *payEnd = strrchr(payStart, '"');
  if (!payEnd || payEnd < payStart) return false;
  size_t payLen = (size_t)(payEnd - payStart);
  if (payLen >= kMqttPayloadLen) payLen = kMqttPayloadLen - 1;

  out.clientIdx = (int)idx;
  out.msgId = (int)msg;
  memcpy(out.topic, topicStart, topicLen);
  out.topic[topicLen] = '\0';
  memcpy(out.payload, payStart, payLen);
  out.payload[payLen] = '\0';
  return true;
}

namespace {

// Devuelve un puntero justo despues del ':' que sigue a "<clave>", o nullptr.
const char *jsonSeekValue(const char *json, const char *key) {
  char quoted[40];
  size_t n = strlen(key);
  if (n + 3 > sizeof(quoted)) return nullptr;
  quoted[0] = '"';
  memcpy(quoted + 1, key, n);
  quoted[n + 1] = '"';
  quoted[n + 2] = '\0';

  const char *at = strstr(json, quoted);
  if (!at) return nullptr;
  const char *p = at + n + 2;
  while (*p == ' ') p++;
  if (*p != ':') return nullptr;
  p++;
  while (*p == ' ') p++;
  return p;
}

}  // namespace

bool jsonScalar(const char *json, const char *key, char *out, size_t cap) {
  const char *p = jsonSeekValue(json, key);
  if (!p) return false;
  if (*p == '"') return jsonString(json, key, out, cap);

  // Token desnudo: numero, true, false o null. Termina en el primer separador.
  const char *e = p;
  while (*e && *e != ',' && *e != '}' && *e != ']' &&
         *e != ' ' && *e != '\t' && *e != '\r' && *e != '\n') {
    e++;
  }
  const size_t len = (size_t)(e - p);
  if (len == 0 || len >= cap) return false;
  memcpy(out, p, len);
  out[len] = '\0';
  return true;
}

bool jsonString(const char *json, const char *key, char *out, size_t cap) {
  const char *p = jsonSeekValue(json, key);
  if (!p || *p != '"') return false;
  p++;
  const char *e = strchr(p, '"');
  if (!e) return false;
  size_t len = (size_t)(e - p);
  if (len >= cap) len = cap - 1;
  memcpy(out, p, len);
  out[len] = '\0';
  return true;
}

bool jsonInt(const char *json, const char *key, long &out) {
  const char *p = jsonSeekValue(json, key);
  if (!p) return false;
  if (*p == '"') p++;  // aceptar tambien un numero entre comillas
  char *end = nullptr;
  long v = strtol(p, &end, 10);
  if (end == p) return false;
  out = v;
  return true;
}

bool jsonObject(const char *json, const char *key, char *out, size_t cap) {
  const char *p = jsonSeekValue(json, key);
  if (!p || *p != '{') return false;
  int depth = 0;
  const char *start = p;
  bool inStr = false;
  for (; *p; p++) {
    if (inStr) {
      if (*p == '\\' && p[1]) p++;
      else if (*p == '"') inStr = false;
      continue;
    }
    if (*p == '"') inStr = true;
    else if (*p == '{') depth++;
    else if (*p == '}') {
      depth--;
      if (depth == 0) {
        size_t len = (size_t)(p - start) + 1;
        if (len >= cap) len = cap - 1;
        memcpy(out, start, len);
        out[len] = '\0';
        return true;
      }
    }
  }
  return false;
}

bool jsonNextPair(const char *&cursor, char *key, size_t keyCap, char *value, size_t valueCap) {
  const char *p = cursor;
  // avanzar hasta la siguiente clave entre comillas
  while (*p && *p != '"') {
    if (*p == '}') return false;  // fin de este objeto
    p++;
  }
  if (*p != '"') return false;
  p++;
  const char *ke = strchr(p, '"');
  if (!ke) return false;
  size_t klen = (size_t)(ke - p);
  if (klen >= keyCap) klen = keyCap - 1;
  memcpy(key, p, klen);
  key[klen] = '\0';

  p = ke + 1;
  while (*p == ' ') p++;
  if (*p != ':') return false;
  p++;
  while (*p == ' ') p++;

  if (*p == '"') {
    p++;
    const char *ve = strchr(p, '"');
    if (!ve) return false;
    size_t vlen = (size_t)(ve - p);
    if (vlen >= valueCap) vlen = valueCap - 1;
    memcpy(value, p, vlen);
    value[vlen] = '\0';
    cursor = ve + 1;
  } else {
    const char *ve = p;
    while (*ve && *ve != ',' && *ve != '}' && *ve != ' ') ve++;
    size_t vlen = (size_t)(ve - p);
    if (vlen >= valueCap) vlen = valueCap - 1;
    memcpy(value, p, vlen);
    value[vlen] = '\0';
    cursor = ve;
  }
  return true;
}

// ---- HISTORICO: via UDP (ya no la usa la via MQTT; solo la prueba unitaria) ----

bool parseDownlink(const char *text, DownlinkAck &out) {
  out = DownlinkAck{};
  if (strncmp(text, "ACK,", 4) != 0) return false;

  const char *p = text + 4;
  char *end = nullptr;
  long seq = strtol(p, &end, 10);
  if (end == p || seq < 0) return false;
  out.seq = (uint16_t)seq;
  out.valid = true;

  p = end;
  if (*p != ',') return true;  // "ACK,<seq>" a secas: valido, sin CFG
  p++;
  if (strncmp(p, "CFG,", 4) != 0) return true;  // sufijo desconocido: el ACK en si sigue valiendo
  p += 4;

  long version = strtol(p, &end, 10);
  if (end == p || version < 0) return true;  // cabecera CFG mal formada: se ignora solo la parte CFG
  out.cfgVersion = (uint8_t)version;
  out.hasCfg = true;
  p = end;

  while (*p == ',' && out.kvCount < kMaxDownlinkKv) {
    p++;
    const char *eq = strchr(p, '=');
    if (!eq) break;
    const char *nextComma = strchr(eq, ',');
    size_t keyLen = (size_t)(eq - p);
    size_t valLen = nextComma ? (size_t)(nextComma - eq - 1) : strlen(eq + 1);

    if (keyLen > 0 && keyLen < kDownlinkKeyLen && valLen > 0 && valLen < kDownlinkValueLen) {
      DownlinkKv &slot = out.kv[out.kvCount++];
      memcpy(slot.key, p, keyLen);
      slot.key[keyLen] = '\0';
      memcpy(slot.value, eq + 1, valLen);
      slot.value[valLen] = '\0';
    }
    // Un elemento mal formado o demasiado largo simplemente no se guarda; el resto de
    // la linea merece la pena leerlo, asi que se avanza p en cualquier caso.
    p = nextComma ? nextComma : eq + 1 + valLen;
  }
  return true;
}

void encodeBatchHeader(uint16_t seq, uint8_t count, uint8_t cfgVersion, uint8_t *out) {
  out[0] = kBatchMagic;
  putU16BE(out + 1, seq);
  out[3] = count;
  out[4] = cfgVersion;
}

void encodeBatchRecord(const SensorSnapshot &snap, uint16_t dtSec, uint8_t *out) {
  putU16BE(out + 0, dtSec);
  putU16BE(out + 2, (uint16_t)clampToInt16((long)snap.rssi));
  out[4] = (uint8_t)(int8_t)snap.snr;
  putU16BE(out + 5, (uint16_t)clampToInt16(lroundf(snap.temp * 10.0f)));
  putU16BE(out + 7, clampToUint16(lroundf(snap.hum * 10.0f)));
  putU16BE(out + 9, clampToUint16(lroundf(snap.pres * 10.0f)));
  putU16BE(out + 11, clampToUint16(lroundf(snap.gas / 100.0f)));
  putU16BE(out + 13, clampToUint16(lroundf(snap.pm1 * 10.0f)));
  putU16BE(out + 15, clampToUint16(lroundf(snap.pm25 * 10.0f)));
  putU16BE(out + 17, clampToUint16(lroundf(snap.pm10 * 10.0f)));
  putU16BE(out + 19, clampToUint16(lroundf(snap.co2)));
  putU16BE(out + 21, clampToUint16(lroundf(snap.co * 100.0f)));
  putU16BE(out + 23, (uint16_t)clampToInt16(lroundf(snap.coTemp * 10.0f)));
  putU16BE(out + 25, clampToUint16(lroundf(snap.windAngle * 10.0f)));
  putU16BE(out + 27, clampToUint16(lroundf(snap.windSpeed * 10.0f)));
  out[29] = snap.windValid ? 0x01 : 0x00;
}

}  // namespace NbiotProtocol
