#include "nice-bust4.h"
#include "esphome/core/log.h"
#include "esphome/core/helpers.h"

namespace esphome {
namespace bus_t4 {

static const char *TAG = "bus_t4.cover";

using namespace esphome::cover;

CoverTraits NiceBusT4::get_traits() {
  auto traits = CoverTraits();
  traits.set_supports_position(true);
  traits.set_supports_stop(true);
  return traits;
}

void NiceBusT4::control(const CoverCall &call) {
  position_hook_type = IGNORE;
  if (call.get_stop()) {
    send_cmd(STOP);
  } else if (call.get_position().has_value()) {
    float newpos = *call.get_position();
    if (newpos != position) {
      if (newpos == COVER_OPEN) {
        if (current_operation != COVER_OPERATION_OPENING) send_cmd(OPEN);
      } else if (newpos == COVER_CLOSED) {
        if (current_operation != COVER_OPERATION_CLOSING) send_cmd(CLOSE);
      } else {
        position_hook_value = (_pos_opn - _pos_cls) * newpos + _pos_cls;
        ESP_LOGI(TAG, "Requested position: %d", position_hook_value);
        if (position_hook_value > _pos_usl) {
          position_hook_type = STOP_UP;
          if (current_operation != COVER_OPERATION_OPENING) send_cmd(OPEN);
        } else {
          position_hook_type = STOP_DOWN;
          if (current_operation != COVER_OPERATION_CLOSING) send_cmd(CLOSE);
        }
      }
    }
  }
}

void NiceBusT4::setup() {
  _uart = uart_init(_UART_NO, BAUD_WORK, SERIAL_8N1, SERIAL_FULL, TX_P, 256, false);
}

void NiceBusT4::loop() {
  if ((millis() - this->last_update_) > 10000) {
    if (this->init_ok == false) {
      this->tx_buffer_.push(gen_inf_cmd(0x00, 0xff, FOR_ALL, WHO, GET, 0x00));
      this->tx_buffer_.push(gen_inf_cmd(0x00, 0xff, FOR_ALL, PRD, GET, 0x00));
    }
    this->last_update_ = millis();
  }

  uint32_t now = millis();
  if (now - this->last_uart_byte_ > 100) {
    this->ready_to_tx_ = true;
    this->last_uart_byte_ = now;
  }

  while (uart_rx_available(_uart) > 0) {
    uint8_t c = (uint8_t)uart_read_char(_uart);
    this->handle_char_(c);
    this->last_uart_byte_ = now;
  }

  if (this->ready_to_tx_ && !this->tx_buffer_.empty()) {
    this->send_array_cmd(this->tx_buffer_.front());
    this->tx_buffer_.pop();
    this->ready_to_tx_ = false;
  }

  if (!is_robus && init_ok && (current_operation != COVER_OPERATION_IDLE) && 
      (now - last_position_time > POSITION_UPDATE_INTERVAL)) {
    last_position_time = now;
    request_position();
  }
}

void NiceBusT4::handle_char_(uint8_t c) {
  this->rx_message_.push_back(c);
  if (!this->validate_message_()) {
    this->rx_message_.clear();
  }
}

bool NiceBusT4::validate_message_() {
  uint32_t at = this->rx_message_.size() - 1;
  uint8_t *data = &this->rx_message_[0];
  uint8_t new_byte = data[at];

  // Basic header validation
  if (at == 0) return new_byte == 0x00;
  if (at == 1) return new_byte == START_CODE;
  if (at == 2) return true;

  uint8_t packet_size = data[2];
  uint8_t length = (packet_size + 3);

  if (at <= 8) return true;

  // Improved CRC1 calculation
  uint8_t crc1 = data[3];
  for (int i = 4; i <= 8; i++) {
    crc1 ^= data[i];
  }

  if (at == 9) {
    if (data[9] != crc1) {
      ESP_LOGW(TAG, "Invalid CRC1: calc=%02X recv=%02X", crc1, data[9]);
      ESP_LOGW(TAG, "Message: %s", format_hex_pretty(rx_message_).c_str());
      return false;
    }
    return true;
  }

  if (at < length) return true;

  // CRC2 calculation
  uint8_t crc2 = data[10];
  for (uint8_t i = 11; i < length - 1; i++) {
    crc2 ^= data[i];
  }

  if (data[length - 1] != crc2) {
    ESP_LOGW(TAG, "Invalid CRC2: calc=%02X recv=%02X", crc2, data[length - 1]);
    return false;
  }

  if (data[length] != packet_size) {
    ESP_LOGW(TAG, "Invalid size: exp=%02X recv=%02X", packet_size, data[length]);
    return false;
  }

  // Process valid message
  rx_message_.erase(rx_message_.begin());
  ESP_LOGD(TAG, "Received: %s", format_hex_pretty(rx_message_).c_str());
  parse_status_packet(rx_message_);

  return false;
}

// [Остальные функции остаются без изменений...]
// parse_status_packet(), dump_config(), gen_control_cmd(), gen_inf_cmd(),
// send_raw_cmd(), raw_cmd_prepare(), send_array_cmd(), send_inf_cmd(),
// set_mcu(), init_device(), request_position(), update_position(),
// publish_state_if_changed()

}  // namespace bus_t4
}  // namespace esphome
