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
  _uart = uart_init(_UART_NO, BAUD_WORK, SERIAL_8N1, SERIAL_FULL, TX_P, 256, true);
  
  // Reset bus
  uart_set_baudrate(_uart, BAUD_BREAK);
  uart_write(_uart, "\x00", 1);
  uart_flush(_uart);
  delay(50);
  uart_set_baudrate(_uart, BAUD_WORK);
  
  // Initial discovery
  this->tx_buffer_.push(gen_inf_cmd(0x00, 0xFF, FOR_ALL, WHO, GET, 0x00));
}

void NiceBusT4::loop() {
  // Periodic device discovery
  if ((millis() - this->last_update_) > 10000 && !this->init_ok) {
    this->tx_buffer_.push(gen_inf_cmd(0x00, 0xFF, FOR_ALL, WHO, GET, 0x00));
    this->last_update_ = millis();
  }

  // Send queued commands
  uint32_t now = millis();
  if (now - this->last_uart_byte_ > 100 && !this->tx_buffer_.empty()) {
    this->send_array_cmd(this->tx_buffer_.front());
    this->tx_buffer_.pop();
    this->last_uart_byte_ = now;
  }

  // Receive handling
  while (uart_rx_available(_uart) {
    uint8_t c = uart_read_char(_uart);
    this->handle_char_(c);
    this->last_uart_byte_ = now;
  }

  // Position updates
  if (!is_robus && init_ok && current_operation != COVER_OPERATION_IDLE && 
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
  if (rx_message_.empty()) return false;
  
  // Minimum message check
  if (rx_message_.size() < 5) {
    ESP_LOGW(TAG, "Message too short: %d bytes", rx_message_.size());
    return false;
  }

  // Header validation
  if (rx_message_[0] != 0x00 || rx_message_[1] != START_CODE) {
    ESP_LOGW(TAG, "Invalid header: %02X %02X", rx_message_[0], rx_message_[1]);
    return false;
  }

  uint8_t packet_size = rx_message_[2];
  uint8_t expected_length = packet_size + 3;

  // Wait for complete message
  if (rx_message_.size() < expected_length) {
    return true;
  }

  // CRC1 calculation (bytes 3-8)
  uint8_t crc1 = rx_message_[3];
  for (int i = 4; i <= 8 && i < rx_message_.size(); i++) {
    crc1 ^= rx_message_[i];
  }

  if (rx_message_[9] != crc1) {
    ESP_LOGW(TAG, "CRC1 mismatch: calc=%02X recv=%02X Packet: %s", 
            crc1, rx_message_[9], format_hex_pretty(rx_message_).c_str());
    return false;
  }

  // CRC2 calculation (bytes 10 to length-2)
  uint8_t crc2 = rx_message_[10];
  for (uint8_t i = 11; i < expected_length - 1; i++) {
    crc2 ^= rx_message_[i];
  }

  if (rx_message_[expected_length - 1] != crc2) {
    ESP_LOGW(TAG, "CRC2 mismatch: calc=%02X recv=%02X", crc2, rx_message_[expected_length - 1]);
    return false;
  }

  // Process valid message
  rx_message_.erase(rx_message_.begin());
  ESP_LOGD(TAG, "Received: %s", format_hex_pretty(rx_message_).c_str());
  parse_status_packet(rx_message_);
  return false;
}

void NiceBusT4::send_array_cmd(const uint8_t *data, size_t len) {
  // BREAK condition
  uart_set_baudrate(_uart, BAUD_BREAK);
  uart_write(_uart, "\x00", 1);
  uart_flush(_uart);
  delayMicroseconds(1000);
  
  // Main transmission
  uart_set_baudrate(_uart, BAUD_WORK);
  delayMicroseconds(100);
  uart_write(_uart, (char *)data, len);
  uart_flush(_uart);
  
  ESP_LOGD(TAG, "Sent: %s", format_hex_pretty(data, len).c_str());
}

// Остальные методы остаются без изменений
void NiceBusT4::parse_status_packet(const std::vector<uint8_t> &data) {
  // ... существующая реализация ...
}

void NiceBusT4::dump_config() {
  // ... существующая реализация ...
}

std::vector<uint8_t> NiceBusT4::gen_control_cmd(const uint8_t control_cmd) {
  // ... существующая реализация ...
}

std::vector<uint8_t> NiceBusT4::gen_inf_cmd(const uint8_t to_addr1, const uint8_t to_addr2, 
                                          const uint8_t whose, const uint8_t inf_cmd, 
                                          const uint8_t run_cmd, const uint8_t next_data, 
                                          const std::vector<uint8_t> &data, size_t len) {
  // ... существующая реализация ...
}

void NiceBusT4::send_raw_cmd(std::string data) {
  // ... существующая реализация ...
}

std::vector<uint8_t> NiceBusT4::raw_cmd_prepare(std::string data) {
  // ... существующая реализация ...
}

void NiceBusT4::send_inf_cmd(std::string to_addr, std::string whose, 
                            std::string command, std::string type_command, 
                            std::string next_data, bool data_on, 
                            std::string data_command) {
  // ... существующая реализация ...
}

void NiceBusT4::set_mcu(std::string command, std::string data_command) {
  // ... существующая реализация ...
}

void NiceBusT4::init_device(const uint8_t addr1, const uint8_t addr2, const uint8_t device) {
  // ... существующая реализация ...
}

void NiceBusT4::request_position(void) {
  // ... существующая реализация ...
}

void NiceBusT4::update_position(uint16_t newpos) {
  // ... существующая реализация ...
}

void NiceBusT4::publish_state_if_changed(void) {
  // ... существующая реализация ...
}

}  // namespace bus_t4
}  // namespace esphome
