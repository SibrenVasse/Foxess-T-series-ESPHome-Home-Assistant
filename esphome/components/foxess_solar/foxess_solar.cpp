#include <algorithm>

#include "foxess_solar.h"
#include "esphome/core/log.h"
#include "esphome/core/helpers.h"

namespace esphome {
namespace foxess_solar {

void publish_sensor_state(sensor::Sensor *sensor, int32_t val, float unit) {
  if (sensor == nullptr)
    return;
  float value = val * unit;
  sensor->publish_state(value);
};

int16_t encode_int16(uint8_t msb, uint8_t lsb) { return (int16_t) (msb << 8 | lsb); }

void FoxessSolar::setup() {
  ESP_LOGV("FoxessSolar::setup", "start");
  this->flow_control_pin_->setup();

  this->millis_lastmessage_ = millis();
  this->input_buffer.reserve(165);
}

void FoxessSolar::update() {
  ESP_LOGV("FoxessSolar::update", "start");
  if (millis() - this->millis_lastmessage_ >= INVERTER_TIMEOUT) {
    if (this->inverter_mode_ != 0) {
      this->set_inverter_mode(0);  // OFFLINE
      publish_sensor_state(this->generation_power_, 0, 1);
      publish_sensor_state(this->grid_power_, 0, NAN);
      publish_sensor_state(this->loads_power_, 0, NAN);

      publish_sensor_state(this->boost_temp_, 0, NAN);
      publish_sensor_state(this->ambient_temp_, 0, NAN);
      publish_sensor_state(this->inverter_temp_, 0, NAN);

      PUBLISH_ZERO_PHASE(0, 1, 2)
      PUBLISH_ZERO_PV(0, 1, 2)
    }
  }

  while (this->available() > 0) {
    this->input_buffer.push_back(this->read());

    if (this->input_buffer.size() < 9) {  // After 9 bytes total_message_length is available
      continue;
    }

    if (this->input_buffer[0] != 0x7E || this->input_buffer[1] != 0x7E || this->input_buffer[2] != 0x02) {
      ESP_LOGV("FoxessSolar::update", "start of message incorrect: 0x%x, 0x%x, 0x%x", this->input_buffer[0],
               this->input_buffer[1], this->input_buffer[2]);
      this->input_buffer.erase(this->input_buffer.begin());
      continue;
    }

    uint16_t total_message_length = encode_uint16(this->input_buffer[7], this->input_buffer[8]) + 13;
    if (this->input_buffer.size() < total_message_length) {
      ESP_LOGV("FoxessSolar::update", "message not ready, size: %d, input_buff: %d", total_message_length,
               this->input_buffer.size());
      continue;
    }

    if (this->input_buffer[total_message_length - 1] != 0xE7 || this->input_buffer[total_message_length - 2] != 0xE7) {
      ESP_LOGE("FoxessSolar::update", "Message footer incorrect [..., 0x%x, 0x%x]",
               this->input_buffer[total_message_length - 2], this->input_buffer[total_message_length - 1]);
      this->input_buffer.clear();
      continue;
    }

    uint16_t received_crc = crc16(this->input_buffer.data() + 2,   // Data start after header size 2
                                  this->input_buffer.size() - 6);  // -2 HEAD -2 FOOT -2 CHECKSUM
    uint16_t calc_crc =
        encode_uint16(this->input_buffer[total_message_length - 3], this->input_buffer[total_message_length - 4]);
    if (received_crc != calc_crc) {
      ESP_LOGE("FoxessSolar::update", "Checksum mismatch, calc: 0x%x, message: 0x%x", calc_crc, received_crc);
      this->input_buffer.clear();
      continue;
    }

    this->parse_message();
    this->input_buffer.clear();
  }
}

void FoxessSolar::parse_message() {
  ESP_LOGV("FoxessSolar::parse_message", "start");
  this->millis_lastmessage_ = millis();

  auto &msg = this->input_buffer;
  publish_sensor_state(this->grid_power_, encode_int16(msg[9], msg[10]), 1);
  publish_sensor_state(this->generation_power_, encode_uint16(msg[11], msg[12]), 1);
  publish_sensor_state(this->loads_power_, encode_int16(msg[13], msg[14]), 1);

  publish_sensor_state(this->phases_[0].voltage_sensor_, encode_uint16(msg[15], msg[16]), 0.1);
  publish_sensor_state(this->phases_[0].current_sensor_, encode_uint16(msg[17], msg[18]), 0.1);
  publish_sensor_state(this->phases_[0].frequency_sensor_, encode_uint16(msg[19], msg[20]), 0.01);
  publish_sensor_state(this->phases_[0].active_power_sensor_, encode_uint16(msg[21], msg[22]), 1);

  publish_sensor_state(this->phases_[1].voltage_sensor_, encode_uint16(msg[23], msg[24]), 0.1);
  publish_sensor_state(this->phases_[1].current_sensor_, encode_uint16(msg[25], msg[26]), 0.1);
  publish_sensor_state(this->phases_[1].frequency_sensor_, encode_uint16(msg[27], msg[28]), 0.01);
  publish_sensor_state(this->phases_[1].active_power_sensor_, encode_uint16(msg[29], msg[30]), 1);

  publish_sensor_state(this->phases_[2].voltage_sensor_, encode_uint16(msg[31], msg[32]), 0.1);
  publish_sensor_state(this->phases_[2].current_sensor_, encode_uint16(msg[33], msg[34]), 0.1);
  publish_sensor_state(this->phases_[2].frequency_sensor_, encode_uint16(msg[35], msg[36]), 0.01);
  publish_sensor_state(this->phases_[2].active_power_sensor_, encode_uint16(msg[37], msg[38]), 1);

  uint16_t volt = encode_uint16(msg[39], msg[40]);
  uint16_t amps = encode_uint16(msg[41], msg[42]);
  publish_sensor_state(this->pvs_[0].voltage_sensor_, volt, 0.1);
  publish_sensor_state(this->pvs_[0].current_sensor_, amps, 0.1);
  publish_sensor_state(this->pvs_[0].active_power_sensor_, volt * amps, 0.01);

  volt = encode_uint16(msg[45], msg[46]);
  amps = encode_uint16(msg[47], msg[48]);
  publish_sensor_state(this->pvs_[1].voltage_sensor_, volt, 0.1);
  publish_sensor_state(this->pvs_[1].current_sensor_, amps, 0.1);
  publish_sensor_state(this->pvs_[1].active_power_sensor_, volt * amps, 0.01);

  volt = encode_uint16(msg[51], msg[52]);
  amps = encode_uint16(msg[53], msg[54]);
  publish_sensor_state(this->pvs_[2].voltage_sensor_, volt, 0.1);
  publish_sensor_state(this->pvs_[2].current_sensor_, amps, 0.1);
  publish_sensor_state(this->pvs_[2].active_power_sensor_, volt * amps, 0.01);

  volt = encode_uint16(msg[57], msg[58]);
  amps = encode_uint16(msg[59], msg[60]);
  publish_sensor_state(this->pvs_[3].voltage_sensor_, volt, 0.1);
  publish_sensor_state(this->pvs_[3].current_sensor_, amps, 0.1);
  publish_sensor_state(this->pvs_[3].active_power_sensor_, volt * amps, 0.01);

  publish_sensor_state(this->boost_temp_, encode_int16(msg[63], msg[64]), 1);
  publish_sensor_state(this->inverter_temp_, encode_int16(msg[65], msg[66]), 1);
  publish_sensor_state(this->ambient_temp_, encode_int16(msg[67], msg[68]), 1);

  publish_sensor_state(this->energy_production_day_, encode_uint16(msg[69], msg[70]), 0.1);

  publish_sensor_state(this->total_energy_production_, encode_uint32(msg[71], msg[72], msg[73], msg[74]), 0.1);

  if (!std::all_of(this->input_buffer.begin() + 125, this->input_buffer.begin() + 157, [](int i) { return i == 0; })) {
    this->set_inverter_mode(2);  // ERROR
    return;
  }

  this->set_inverter_mode(1);  // ONLINE
}

void FoxessSolar::set_inverter_mode(uint32_t mode) {
  this->inverter_mode_ = mode;
  if (this->inverter_status_ != nullptr)
    this->inverter_status_->publish_state(mode);
}

}  // namespace foxess_solar
}  // namespace esphome
