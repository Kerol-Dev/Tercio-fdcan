// Host-side tests for the adapter's framing and protocol.
// Run with: pio test -e native
#include <unity.h>

#include <cstring>
#include <vector>

#include "bridge/AdapterProtocol.h"
#include "bridge/Framing.h"

namespace {

std::vector<uint8_t> wire(uint16_t id, uint8_t opcode, const std::vector<uint8_t>& payload) {
  std::vector<uint8_t> out(framing::kMaxEncoded);
  out.resize(framing::encode(id, opcode, payload.data(), static_cast<uint8_t>(payload.size()), out.data()));
  return out;
}

std::vector<framing::Frame> feed(framing::Parser& parser, const std::vector<uint8_t>& bytes) {
  std::vector<framing::Frame> frames;
  framing::Frame frame;
  for (uint8_t b : bytes)
    if (parser.push(b, frame)) frames.push_back(frame);
  return frames;
}

}  // namespace

void test_crc16_check_value() {
  const uint8_t check[] = {'1', '2', '3', '4', '5', '6', '7', '8', '9'};
  TEST_ASSERT_EQUAL_HEX16(0x29B1, framing::crc16(check, sizeof check));
}

void test_cobs_known_vectors() {
  const uint8_t a[] = {0x00};
  const uint8_t b[] = {0x11, 0x22, 0x00, 0x33};
  const uint8_t c[] = {0x11, 0x00, 0x00, 0x00};
  uint8_t out[8];
  TEST_ASSERT_EQUAL_UINT8(2, framing::cobsEncode(a, sizeof a, out));
  TEST_ASSERT_EQUAL_HEX8(0x01, out[0]);
  TEST_ASSERT_EQUAL_HEX8(0x01, out[1]);
  TEST_ASSERT_EQUAL_UINT8(5, framing::cobsEncode(b, sizeof b, out));
  const uint8_t eb[] = {0x03, 0x11, 0x22, 0x02, 0x33};
  TEST_ASSERT_EQUAL_MEMORY(eb, out, 5);
  TEST_ASSERT_EQUAL_UINT8(5, framing::cobsEncode(c, sizeof c, out));
  const uint8_t ec[] = {0x02, 0x11, 0x01, 0x01, 0x01};
  TEST_ASSERT_EQUAL_MEMORY(ec, out, 5);
  uint8_t back[8];
  TEST_ASSERT_EQUAL_INT(4, framing::cobsDecode(out, 5, back));
  TEST_ASSERT_EQUAL_MEMORY(c, back, 4);
}

void test_round_trip_all_lengths_with_zeros() {
  framing::Parser parser;
  for (uint8_t length = 0; length <= framing::kMaxPayload; ++length) {
    std::vector<uint8_t> payload(length);
    for (uint8_t i = 0; i < length; ++i) payload[i] = static_cast<uint8_t>(i % 3 ? i : 0);  // plenty of zeros
    const auto bytes = wire(0x105, 0x00, payload);
    TEST_ASSERT_EQUAL_HEX8(0x00, bytes.back());
    for (size_t i = 0; i + 1 < bytes.size(); ++i) TEST_ASSERT_NOT_EQUAL(0, bytes[i]);  // only the delimiter is zero
    const auto frames = feed(parser, bytes);
    TEST_ASSERT_EQUAL(1, frames.size());
    TEST_ASSERT_EQUAL_HEX16(0x105, frames[0].id);
    TEST_ASSERT_EQUAL_UINT8(length, frames[0].length);
    if (length) TEST_ASSERT_EQUAL_MEMORY(payload.data(), frames[0].payload, length);
  }
  TEST_ASSERT_EQUAL_UINT32(0, parser.rejectedFrames());
}

void test_mid_stream_start_resynchronises() {
  // The host opens the port in the middle of a frame: the tail is rejected,
  // the next frame is intact.
  framing::Parser parser;
  const auto a = wire(0x201, 0x01, std::vector<uint8_t>(40, 0x5A));
  const auto b = wire(0x202, 0x01, {1, 2, 3});
  std::vector<uint8_t> stream(a.begin() + 17, a.end());
  stream.insert(stream.end(), b.begin(), b.end());
  const auto frames = feed(parser, stream);
  TEST_ASSERT_EQUAL(1, frames.size());
  TEST_ASSERT_EQUAL_HEX16(0x202, frames[0].id);
  TEST_ASSERT_EQUAL_UINT32(1, parser.rejectedFrames());
}

void test_corruption_is_rejected() {
  framing::Parser parser;
  auto bytes = wire(0x185, 0x09, {0x22, 1, 2, 3, 4});
  bytes[3] ^= 0x10;  // flip a bit
  TEST_ASSERT_EQUAL(0, feed(parser, bytes).size());
  TEST_ASSERT_EQUAL_UINT32(1, parser.rejectedFrames());
  // Garbage longer than any frame, then a good one.
  std::vector<uint8_t> junk(200, 0x77);
  junk.push_back(0);
  const auto good = wire(0x185, 0x09, {7});
  junk.insert(junk.end(), good.begin(), good.end());
  const auto frames = feed(parser, junk);
  TEST_ASSERT_EQUAL(1, frames.size());
  TEST_ASSERT_EQUAL_UINT8(7, frames[0].payload[0]);
}

void test_ids() {
  framing::Parser parser;
  TEST_ASSERT_EQUAL(1, feed(parser, wire(framing::kAdapterId, 0x01, {})).size());
  TEST_ASSERT_EQUAL(1, feed(parser, wire(0x7FF, 0x01, {})).size());
  TEST_ASSERT_EQUAL(0, feed(parser, wire(0x801, 0x01, {})).size());  // neither CAN nor adapter
  TEST_ASSERT_EQUAL(0, feed(parser, {0x00, 0x00, 0x00}).size());  // empty packets: ignored
}

void test_fd_length_mapping() {
  for (uint8_t n = 0; n <= 8; ++n) TEST_ASSERT_EQUAL_UINT8(n, framing::dlcFor(n));
  TEST_ASSERT_EQUAL_UINT8(9, framing::dlcFor(9));
  TEST_ASSERT_EQUAL_UINT8(14, framing::dlcFor(33));
  TEST_ASSERT_EQUAL_UINT8(15, framing::dlcFor(64));
  for (uint8_t n = 0; n <= 64; ++n) TEST_ASSERT_TRUE(framing::kFdLengths[framing::dlcFor(n)] >= n);
}

void test_status_and_info_layouts() {
  adapter::Status s;
  s.busState = adapter::BusState::ErrorPassive;
  s.tec = 130;
  s.toCan = 0x11223344;
  s.busOffEvents = 0x0102;
  uint8_t out[64];
  TEST_ASSERT_EQUAL_UINT8(adapter::kStatusSize, adapter::encodeStatus(s, out));
  TEST_ASSERT_EQUAL_UINT8(2, out[0]);
  TEST_ASSERT_EQUAL_UINT8(130, out[1]);
  TEST_ASSERT_EQUAL_UINT8(0x44, out[4]);
  TEST_ASSERT_EQUAL_UINT8(0x11, out[7]);
  TEST_ASSERT_EQUAL_UINT8(0x02, out[24]);
  TEST_ASSERT_EQUAL_UINT8(0x01, out[25]);

  adapter::Info info;
  info.fwMajor = 2;
  info.flags = adapter::kInfoCrystalClock;
  for (uint8_t i = 0; i < 12; ++i) info.uid[i] = i;
  TEST_ASSERT_EQUAL_UINT8(adapter::kInfoSize, adapter::encodeInfo(info, out));
  TEST_ASSERT_EQUAL_UINT8(adapter::kProtocolVersion, out[0]);
  TEST_ASSERT_EQUAL_UINT8(adapter::kInfoCrystalClock, out[5]);
  TEST_ASSERT_EQUAL_UINT8(11, out[19]);
}

int main(int, char**) {
  UNITY_BEGIN();
  RUN_TEST(test_crc16_check_value);
  RUN_TEST(test_cobs_known_vectors);
  RUN_TEST(test_round_trip_all_lengths_with_zeros);
  RUN_TEST(test_mid_stream_start_resynchronises);
  RUN_TEST(test_corruption_is_rejected);
  RUN_TEST(test_ids);
  RUN_TEST(test_fd_length_mapping);
  RUN_TEST(test_status_and_info_layouts);
  return UNITY_END();
}
