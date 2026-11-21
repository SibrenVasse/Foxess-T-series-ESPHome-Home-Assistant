#include "foxess_solar.h"

#include <algorithm>
#include <cmath>

#include "esphome/core/log.h"

namespace esphome {
namespace foxess_solar {

namespace {

static const char *const TAG = "foxess_solar";

// Field starts in the original layout. The shifted layout changes fields from PHASE1_BASE onward.
struct MsgOffset {
  static constexpr std::size_t GRID_POWER = 9;
  static constexpr std::size_t GEN_POWER = 11;
  static constexpr std::size_t LOAD_POWER = 13;
  static constexpr std::size_t SHIFTED_GEN_POWER = 15;
  static constexpr std::size_t FIELD_SHIFT = 6;

  static constexpr std::size_t PHASE1_BASE = 15;
  static constexpr std::size_t PHASE_STRIDE = 8;
  static constexpr std::size_t PV1_BASE = 39;
  static constexpr std::size_t PV_STRIDE = 6;

  static constexpr std::size_t BOOST_TEMP = 63;
  static constexpr std::size_t INVERTER_TEMP = 65;
  static constexpr std::size_t AMBIENT_TEMP = 67;
  static constexpr std::size_t ENERGY_DAY = 69;
  static constexpr std::size_t TOTAL_ENERGY = 71;
  static constexpr std::size_t ERROR_BLOCK_BEGIN = 125;
  static constexpr std::size_t ERROR_BLOCK_END = 157;  // Exclusive.
};

void publish_sensor_state(sensor::Sensor *sensor, float raw, float scale = 1.0f) {
  if (sensor != nullptr) {
    sensor->publish_state(raw * scale);
  }
}

uint16_t decode_uint16(const uint8_t *field) {
  return static_cast<uint16_t>((static_cast<uint16_t>(field[0]) << 8) | field[1]);
}

int16_t decode_int16(const uint8_t *field) { return static_cast<int16_t>(decode_uint16(field)); }

uint32_t decode_uint32(const uint8_t *field) {
  return (static_cast<uint32_t>(field[0]) << 24) | (static_cast<uint32_t>(field[1]) << 16) |
         (static_cast<uint32_t>(field[2]) << 8) | field[3];
}

}  // namespace

void FoxessSolar::setup() {
  ESP_LOGVV(TAG, "setup start");

  if (this->flow_control_pin_ != nullptr) {
    this->flow_control_pin_->setup();
    this->flow_control_pin_->digital_write(false);
  }

  this->last_frame_ms_ = millis();
  publish_sensor_state(this->inverter_status_, static_cast<uint8_t>(this->inverter_mode_));
}

void FoxessSolar::loop() {
  // Read queued bytes first because a delayed loop can resume in the middle of a valid frame.
  uint8_t byte;
  while (this->available() > 0 && this->read_byte(&byte)) {
    this->handle_frame_result(this->parser_.feed(byte, millis()));
  }

  // Evaluate the idle timeout after the parser has seen the queued bytes.
  this->handle_frame_result(this->parser_.expire(millis()));
}

void FoxessSolar::update() {
  // Base the offline transition on the last validated frame and publish it only once.
  const uint32_t now_ms = millis();
  if (this->inverter_mode_ == InverterMode::OFFLINE || now_ms - this->last_frame_ms_ < this->inverter_timeout_ms_) {
    return;
  }

  // Invalidate live readings and retain the last reported energy counters.
  this->set_inverter_mode(InverterMode::OFFLINE);
  publish_sensor_state(this->generation_power_, NAN);
  publish_sensor_state(this->grid_power_, NAN);
  publish_sensor_state(this->loads_power_, NAN);
  publish_sensor_state(this->boost_temp_, NAN);
  publish_sensor_state(this->ambient_temp_, NAN);
  publish_sensor_state(this->inverter_temp_, NAN);

  for (auto &phase : this->phases_) {
    publish_sensor_state(phase.voltage_sensor_, NAN);
    publish_sensor_state(phase.current_sensor_, NAN);
    publish_sensor_state(phase.frequency_sensor_, NAN);
    publish_sensor_state(phase.active_power_sensor_, NAN);
  }
  for (auto &pv : this->pvs_) {
    publish_sensor_state(pv.voltage_sensor_, NAN);
    publish_sensor_state(pv.current_sensor_, NAN);
    publish_sensor_state(pv.active_power_sensor_, NAN);
  }
}

void FoxessSolar::handle_frame_result(FoxessFrameParser::Result result) {
  using Result = FoxessFrameParser::Result;
  switch (result) {
    case Result::WAITING:
      return;
    case Result::FRAME_READY:
      // A validated frame restores communication and clears any prior framing warning.
      this->last_frame_ms_ = millis();
      this->status_clear_warning();
      this->publish_frame(this->parser_.frame());
      return;
    case Result::INVALID_LENGTH:
      ESP_LOGW(TAG, "Unsupported telemetry frame length");
      break;
    case Result::INVALID_FOOTER:
      ESP_LOGW(TAG, "Invalid telemetry frame footer");
      break;
    case Result::INVALID_CHECKSUM:
      ESP_LOGW(TAG, "Telemetry frame checksum mismatch");
      break;
    case Result::TIMED_OUT:
      ESP_LOGW(TAG, "Incomplete telemetry frame timed out");
      break;
  }
  this->status_set_warning();
}

void FoxessSolar::publish_frame(const FoxessFrameParser::Frame &frame) {
  // Select the offset shift once so all fields use the validated frame's layout.
  const uint8_t *msg = frame.data;
  const bool shifted = frame.uses_shifted_layout();
  const std::size_t shift = shifted ? MsgOffset::FIELD_SHIFT : 0;

  // The shifted layout reports generation power before the shifted phase blocks.
  const std::size_t generation_offset = shifted ? MsgOffset::SHIFTED_GEN_POWER : MsgOffset::GEN_POWER;
  publish_sensor_state(this->grid_power_, decode_int16(msg + MsgOffset::GRID_POWER));
  publish_sensor_state(this->generation_power_, decode_uint16(msg + generation_offset));
  publish_sensor_state(this->loads_power_, decode_int16(msg + MsgOffset::LOAD_POWER));

  // Apply the same layout shift and field scaling to each AC phase.
  for (std::size_t i = 0; i < 3; i++) {
    const uint8_t *block = msg + MsgOffset::PHASE1_BASE + shift + i * MsgOffset::PHASE_STRIDE;
    auto &phase = this->phases_[i];
    publish_sensor_state(phase.voltage_sensor_, decode_uint16(block), 0.1f);
    publish_sensor_state(phase.current_sensor_, decode_uint16(block + 2), 0.1f);
    publish_sensor_state(phase.frequency_sensor_, decode_uint16(block + 4), 0.01f);
    publish_sensor_state(phase.active_power_sensor_, decode_uint16(block + 6));
  }

  // The raw PV voltage and current are in tenths. Their product is in hundredths of a watt.
  for (std::size_t i = 0; i < 4; i++) {
    const uint8_t *block = msg + MsgOffset::PV1_BASE + shift + i * MsgOffset::PV_STRIDE;
    const uint16_t volt = decode_uint16(block);
    const uint16_t amps = decode_uint16(block + 2);
    auto &pv = this->pvs_[i];
    publish_sensor_state(pv.voltage_sensor_, volt, 0.1f);
    publish_sensor_state(pv.current_sensor_, amps, 0.1f);
    publish_sensor_state(pv.active_power_sensor_, static_cast<float>(volt) * amps, 0.01f);
  }

  // Decode signed temperatures so readings below zero stay negative.
  publish_sensor_state(this->boost_temp_, decode_int16(msg + MsgOffset::BOOST_TEMP + shift));
  publish_sensor_state(this->inverter_temp_, decode_int16(msg + MsgOffset::INVERTER_TEMP + shift));
  publish_sensor_state(this->ambient_temp_, decode_int16(msg + MsgOffset::AMBIENT_TEMP + shift));

  // Convert both counters from tenths of a kilowatt-hour to kilowatt-hours.
  publish_sensor_state(this->energy_production_day_, decode_uint16(msg + MsgOffset::ENERGY_DAY + shift), 0.1f);
  publish_sensor_state(this->total_energy_production_, decode_uint32(msg + MsgOffset::TOTAL_ENERGY + shift), 0.1f);

  // A nonzero byte in the fault block indicates an inverter fault.
  const bool has_fault = std::any_of(msg + MsgOffset::ERROR_BLOCK_BEGIN + shift,
                                     msg + MsgOffset::ERROR_BLOCK_END + shift, [](uint8_t byte) { return byte != 0; });
  this->set_inverter_mode(has_fault ? InverterMode::FAULT : InverterMode::ONLINE);
}

void FoxessSolar::set_inverter_mode(InverterMode mode) {
  if (mode == this->inverter_mode_) {
    return;
  }
  this->inverter_mode_ = mode;
  publish_sensor_state(this->inverter_status_, static_cast<uint8_t>(mode));
}

}  // namespace foxess_solar
}  // namespace esphome
