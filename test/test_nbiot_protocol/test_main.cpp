// Off-target tests for src/nbiot/NbiotProtocol.* - no Arduino, no HardwareSerial,
// runs on the host: `pio test -e native`. This is the piece ModemNBIoT was
// deliberately kept thin around so it could be exercised without a board.

#include <unity.h>

#include "nbiot/NbiotProtocol.h"

using namespace NbiotProtocol;

void setUp() {}
void tearDown() {}

// ---------- AT response line parsers ----------

void test_parseCereg_query_reply_registered_home() {
  CeregState st;
  TEST_ASSERT_TRUE(parseCereg("+CEREG: 0,1", st));
  TEST_ASSERT_EQUAL(static_cast<int>(CeregState::REGISTERED_HOME), static_cast<int>(st));
  TEST_ASSERT_TRUE(cieregIsAttached(st));
}

void test_parseCereg_roaming_is_attached() {
  CeregState st;
  TEST_ASSERT_TRUE(parseCereg("+CEREG: 2,5", st));
  TEST_ASSERT_TRUE(cieregIsAttached(st));
}

void test_parseCereg_searching_is_not_attached() {
  CeregState st;
  TEST_ASSERT_TRUE(parseCereg("+CEREG: 2,2", st));
  TEST_ASSERT_FALSE(cieregIsAttached(st));
}

void test_parseCereg_rejects_unrelated_line() {
  CeregState st;
  TEST_ASSERT_FALSE(parseCereg("+CSQ: 20,99", st));
}

void test_parseCsq_typical() {
  int dbm;
  TEST_ASSERT_TRUE(parseCsq("+CSQ: 20,99", dbm));
  TEST_ASSERT_EQUAL(-73, dbm);  // -113 + 20*2
}

void test_parseCsq_unknown_is_rejected() {
  int dbm;
  TEST_ASSERT_FALSE(parseCsq("+CSQ: 99,99", dbm));
}

void test_parseCclk_utc_quoted() {
  int64_t ms;
  TEST_ASSERT_TRUE(parseCclk("+CCLK: \"24/01/01,00:00:00+00\"", ms));
  TEST_ASSERT_EQUAL_INT64(1704067200000LL, ms);  // 2024-01-01T00:00:00Z
}

void test_parseCclk_positive_tz_shifts_to_utc() {
  // +08 = 8 * 15min = +2h ahead of UTC; local 00:00 -> UTC 22:00 the day before
  int64_t ms;
  TEST_ASSERT_TRUE(parseCclk("+CCLK: 24/01/01,00:00:00+08", ms));
  TEST_ASSERT_EQUAL_INT64(1704060000000LL, ms);
}

void test_parseCclk_negative_tz_and_time_of_day() {
  int64_t ms;
  TEST_ASSERT_TRUE(parseCclk("+CCLK: \"24/01/01,12:00:00-04\"", ms));
  TEST_ASSERT_EQUAL_INT64(1704114000000LL, ms);  // 1704067200 + 43200(noon) + 3600(-1h tz)
}

void test_parseCclk_no_timezone_field() {
  int64_t ms;
  TEST_ASSERT_TRUE(parseCclk("+CCLK: 24/06/15,12:30:45", ms));
  TEST_ASSERT_EQUAL_INT64(1718454645000LL, ms);
}

void test_parseCclk_rejects_unrelated_line() {
  int64_t ms;
  TEST_ASSERT_FALSE(parseCclk("+CSQ: 20,0", ms));
}

void test_parseCclk_placeholder_year_parses_but_is_implausible() {
  // A modem with no network time answers with a ~2000-2004 placeholder - it
  // must still parse (so the caller can range-check), just yield a value the
  // caller's own epoch floor rejects.
  int64_t ms;
  TEST_ASSERT_TRUE(parseCclk("+CCLK: \"04/01/01,00:00:00+00\"", ms));
  TEST_ASSERT_LESS_THAN_INT64(1672531200000LL, ms);  // < 2023-01-01
}

void test_parseQiopen_success() {
  int connectId, err;
  TEST_ASSERT_TRUE(parseQiopen("+QIOPEN: 0,0", connectId, err));
  TEST_ASSERT_EQUAL(0, connectId);
  TEST_ASSERT_EQUAL(0, err);
}

void test_parseQiopen_error_code() {
  int connectId, err;
  TEST_ASSERT_TRUE(parseQiopen("+QIOPEN: 0,563", connectId, err));
  TEST_ASSERT_EQUAL(563, err);
}

void test_parseQiurcRecv() {
  int connectId, dataLen;
  TEST_ASSERT_TRUE(parseQiurcRecv("+QIURC: \"recv\",0,17", connectId, dataLen));
  TEST_ASSERT_EQUAL(0, connectId);
  TEST_ASSERT_EQUAL(17, dataLen);
}

void test_parseQiurcRecv_rejects_other_urc() {
  int connectId, dataLen;
  TEST_ASSERT_FALSE(parseQiurcRecv("+QIURC: \"pdpdeact\",0", connectId, dataLen));
}

void test_parseQirdHeader() {
  int len;
  TEST_ASSERT_TRUE(parseQirdHeader("+QIRD: 17", len));
  TEST_ASSERT_EQUAL(17, len);
}

// ---------- Downlink grammar ----------

void test_parseDownlink_bare_ack() {
  DownlinkAck ack;
  TEST_ASSERT_TRUE(parseDownlink("ACK,42", ack));
  TEST_ASSERT_TRUE(ack.valid);
  TEST_ASSERT_EQUAL(42, ack.seq);
  TEST_ASSERT_FALSE(ack.hasCfg);
}

void test_parseDownlink_with_cfg() {
  DownlinkAck ack;
  TEST_ASSERT_TRUE(parseDownlink("ACK,7,CFG,3,N=8,T=600", ack));
  TEST_ASSERT_TRUE(ack.valid);
  TEST_ASSERT_EQUAL(7, ack.seq);
  TEST_ASSERT_TRUE(ack.hasCfg);
  TEST_ASSERT_EQUAL(3, ack.cfgVersion);
  TEST_ASSERT_EQUAL(2, ack.kvCount);
  TEST_ASSERT_EQUAL_STRING("N", ack.kv[0].key);
  TEST_ASSERT_EQUAL_STRING("8", ack.kv[0].value);
  TEST_ASSERT_EQUAL_STRING("T", ack.kv[1].key);
  TEST_ASSERT_EQUAL_STRING("600", ack.kv[1].value);
}

void test_parseDownlink_rejects_non_ack() {
  DownlinkAck ack;
  TEST_ASSERT_FALSE(parseDownlink("NAK,1", ack));
}

void test_parseDownlink_malformed_cfg_keeps_ack_valid() {
  // Version field missing/garbage - the bare ACK is still good, CFG just
  // isn't applied. A bad downlink must never look like a bad ACK.
  DownlinkAck ack;
  TEST_ASSERT_TRUE(parseDownlink("ACK,9,CFG,notanumber", ack));
  TEST_ASSERT_TRUE(ack.valid);
  TEST_ASSERT_EQUAL(9, ack.seq);
  TEST_ASSERT_FALSE(ack.hasCfg);
}

void test_parseDownlink_unknown_suffix_keeps_ack_valid() {
  DownlinkAck ack;
  TEST_ASSERT_TRUE(parseDownlink("ACK,3,SOMETHINGELSE", ack));
  TEST_ASSERT_TRUE(ack.valid);
  TEST_ASSERT_EQUAL(3, ack.seq);
  TEST_ASSERT_FALSE(ack.hasCfg);
}

void test_parseDownlink_caps_kv_count() {
  // kMaxDownlinkKv is 4 - a 5th pair must be silently dropped, not overflow.
  DownlinkAck ack;
  TEST_ASSERT_TRUE(parseDownlink("ACK,1,CFG,1,A=1,B=2,C=3,D=4,E=5", ack));
  TEST_ASSERT_EQUAL(kMaxDownlinkKv, ack.kvCount);
}

void test_parseDownlink_oversized_token_is_skipped_not_fatal() {
  // key "N" is fine but this value is longer than kDownlinkValueLen allows -
  // that one pair should be dropped while T=600 right after it still parses.
  DownlinkAck ack;
  TEST_ASSERT_TRUE(
      parseDownlink("ACK,1,CFG,1,N=123456789012345678,T=600", ack));
  TEST_ASSERT_EQUAL(1, ack.kvCount);
  TEST_ASSERT_EQUAL_STRING("T", ack.kv[0].key);
}

// ---------- Uplink batch frame ----------

static uint16_t readU16BE(const uint8_t *p) {
  return (uint16_t)((p[0] << 8) | p[1]);
}

void test_encodeBatchHeader_layout() {
  uint8_t buf[kBatchHeaderSize];
  encodeBatchHeader(0x1234, 5, 9, buf);
  TEST_ASSERT_EQUAL_HEX8(kBatchMagic, buf[0]);
  TEST_ASSERT_EQUAL(0x1234, readU16BE(buf + 1));
  TEST_ASSERT_EQUAL(5, buf[3]);
  TEST_ASSERT_EQUAL(9, buf[4]);
}

void test_encodeBatchRecord_layout_and_scaling() {
  SensorSnapshot snap;
  snap.rssi = -95;
  snap.snr = -3;
  snap.temp = 23.4f;
  snap.hum = 45.6f;
  snap.pres = 1013.2f;
  snap.gas = 125000.0f;  // -> 1250 in units of 100 ohm
  snap.pm1 = 1.2f;
  snap.pm25 = 3.4f;
  snap.pm10 = 5.6f;
  snap.co2 = 410.0f;
  snap.co = 0.05f;
  snap.coTemp = 24.1f;
  snap.windAngle = 180.5f;
  snap.windSpeed = 3.2f;
  snap.windValid = true;

  uint8_t buf[kBatchRecordSize];
  encodeBatchRecord(snap, 90, buf);

  TEST_ASSERT_EQUAL(90, readU16BE(buf + 0));                       // dtSec
  TEST_ASSERT_EQUAL_INT16(-95, (int16_t)readU16BE(buf + 2));       // rssi
  TEST_ASSERT_EQUAL_INT8(-3, (int8_t)buf[4]);                      // snr
  TEST_ASSERT_EQUAL_INT16(234, (int16_t)readU16BE(buf + 5));       // temp*10
  TEST_ASSERT_EQUAL(456, readU16BE(buf + 7));                      // hum*10
  TEST_ASSERT_EQUAL(10132, readU16BE(buf + 9));                    // pres*10
  TEST_ASSERT_EQUAL(1250, readU16BE(buf + 11));                    // gas/100
  TEST_ASSERT_EQUAL(12, readU16BE(buf + 13));                      // pm1*10
  TEST_ASSERT_EQUAL(34, readU16BE(buf + 15));                      // pm25*10
  TEST_ASSERT_EQUAL(56, readU16BE(buf + 17));                      // pm10*10
  TEST_ASSERT_EQUAL(410, readU16BE(buf + 19));                     // co2
  TEST_ASSERT_EQUAL(5, readU16BE(buf + 21));                       // co*100
  TEST_ASSERT_EQUAL_INT16(241, (int16_t)readU16BE(buf + 23));      // coTemp*10
  TEST_ASSERT_EQUAL(1805, readU16BE(buf + 25));                    // windAngle*10
  TEST_ASSERT_EQUAL(32, readU16BE(buf + 27));                      // windSpeed*10
  TEST_ASSERT_EQUAL(0x01, buf[29]);                                // flags: windValid
}

void test_encodeBatchRecord_windInvalid_flag_clear() {
  SensorSnapshot snap;
  snap.windValid = false;
  uint8_t buf[kBatchRecordSize];
  encodeBatchRecord(snap, 0, buf);
  TEST_ASSERT_EQUAL(0x00, buf[29]);
}

void test_default_batch_size_fits_under_payload_budget() {
  size_t worstCase = kBatchHeaderSize + (size_t)NBIOT_CFG_READINGS_MAX * kBatchRecordSize;
  TEST_ASSERT_LESS_OR_EQUAL(NBIOT_PAYLOAD_MAX_BYTES, worstCase);
  size_t defaultCase = kBatchHeaderSize + (size_t)NBIOT_BATCH_DEFAULT_READINGS * kBatchRecordSize;
  TEST_ASSERT_LESS_OR_EQUAL(NBIOT_PAYLOAD_MAX_BYTES, defaultCase);
}

int main(int argc, char **argv) {
  (void)argc;
  (void)argv;
  UNITY_BEGIN();

  RUN_TEST(test_parseCereg_query_reply_registered_home);
  RUN_TEST(test_parseCereg_roaming_is_attached);
  RUN_TEST(test_parseCereg_searching_is_not_attached);
  RUN_TEST(test_parseCereg_rejects_unrelated_line);
  RUN_TEST(test_parseCsq_typical);
  RUN_TEST(test_parseCsq_unknown_is_rejected);
  RUN_TEST(test_parseCclk_utc_quoted);
  RUN_TEST(test_parseCclk_positive_tz_shifts_to_utc);
  RUN_TEST(test_parseCclk_negative_tz_and_time_of_day);
  RUN_TEST(test_parseCclk_no_timezone_field);
  RUN_TEST(test_parseCclk_rejects_unrelated_line);
  RUN_TEST(test_parseCclk_placeholder_year_parses_but_is_implausible);
  RUN_TEST(test_parseQiopen_success);
  RUN_TEST(test_parseQiopen_error_code);
  RUN_TEST(test_parseQiurcRecv);
  RUN_TEST(test_parseQiurcRecv_rejects_other_urc);
  RUN_TEST(test_parseQirdHeader);

  RUN_TEST(test_parseDownlink_bare_ack);
  RUN_TEST(test_parseDownlink_with_cfg);
  RUN_TEST(test_parseDownlink_rejects_non_ack);
  RUN_TEST(test_parseDownlink_malformed_cfg_keeps_ack_valid);
  RUN_TEST(test_parseDownlink_unknown_suffix_keeps_ack_valid);
  RUN_TEST(test_parseDownlink_caps_kv_count);
  RUN_TEST(test_parseDownlink_oversized_token_is_skipped_not_fatal);

  RUN_TEST(test_encodeBatchHeader_layout);
  RUN_TEST(test_encodeBatchRecord_layout_and_scaling);
  RUN_TEST(test_encodeBatchRecord_windInvalid_flag_clear);
  RUN_TEST(test_default_batch_size_fits_under_payload_budget);

  return UNITY_END();
}
