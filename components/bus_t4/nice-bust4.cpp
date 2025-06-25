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

void NiceBusT4::setup() {
  _uart = uart_init(_UART_NO, BAUD_WORK, SERIAL_8N1, SERIAL_FULL, TX_P, 512, true);
  
  // Reset bus line
  uart_set_baudrate(_uart, 300);
  uart_write(_uart, "\x00\x00\x00", 3);
  uart_flush(_uart);
  delay(100);
  uart_set_baudrate(_uart, BAUD_WORK);
  delay(100);
  
  // Initial device discovery
  this->tx_buffer_.push(gen_inf_cmd(0x00, 0xFF, FOR_ALL, WHO, GET, 0x00));
}

void NiceBusT4::loop() {
  uint32_t now = millis();

  // Periodic discovery if not initialized
  if (!init_ok && (now - last_update_) > 10000) {
    this->tx_buffer_.push(gen_inf_cmd(0x00, 0xFF, FOR_ALL, WHO, GET, 0x00));
    last_update_ = now;
  }

  // Process received data
  while (uart_rx_available(_uart)) {
    uint8_t c = uart_read_char(_uart);
    this->handle_char_(c);
    last_uart_byte_ = now;
  }

  // Send queued commands
  if (ready_to_tx_ && !tx_buffer_.empty() && (now - last_uart_byte_ > 100)) {
    send_array_cmd(tx_buffer_.front());
    tx_buffer_.pop();
    ready_to_tx_ = false;
    last_uart_byte_ = now;
  }

  // Request position updates
  if (!is_robus && init_ok && (current_operation != COVER_OPERATION_IDLE) && 
      (now - last_position_time > POSITION_UPDATE_INTERVAL)) {
    request_position();
    last_position_time = now;
  }
}

void NiceBusT4::control(const CoverCall &call) {
  if (call.get_stop()) {
    send_cmd(STOP);
    position_hook_type = IGNORE;
    return;
  }

  if (call.get_position().has_value()) {
    float newpos = *call.get_position();
    if (newpos != position) {
      if (newpos == COVER_OPEN) {
        send_cmd(OPEN);
      } else if (newpos == COVER_CLOSED) {
        send_cmd(CLOSE);
      } else {
        position_hook_value = (_pos_opn - _pos_cls) * newpos + _pos_cls;
        ESP_LOGD(TAG, "Target position: %d", position_hook_value);
        
        if (position_hook_value > _pos_usl) {
          position_hook_type = STOP_UP;
          send_cmd(OPEN);
        } else {
          position_hook_type = STOP_DOWN;
          send_cmd(CLOSE);
        }
      }
    }
  }
}

void NiceBusT4::handle_char_(uint8_t c) {
  rx_message_.push_back(c);
  if (!validate_message_()) {
    rx_message_.clear();
  }
}

bool NiceBusT4::validate_message_() {
  if (rx_message_.empty()) return false;

  // Minimum message length check
  if (rx_message_.size() < 5) return true;

  // Header validation
  if (rx_message_[0] != 0x00 || rx_message_[1] != START_CODE) {
    ESP_LOGW(TAG, "Invalid header: %02X %02X", rx_message_[0], rx_message_[1]);
    return false;
  }

  uint8_t packet_size = rx_message_[2];
  uint8_t expected_length = packet_size + 3;

  // Wait for complete message
  if (rx_message_.size() < expected_length) return true;

  // Calculate CRC1 (bytes 3-8)
  uint8_t crc1 = rx_message_[3];
  for (int i = 4; i <= 8; i++) {
    crc1 ^= rx_message_[i];
  }

  if (rx_message_[9] != crc1) {
    ESP_LOGW(TAG, "CRC1 error: calc=%02X recv=%02X", crc1, rx_message_[9]);
    ESP_LOGW(TAG, "Packet: %s", format_hex_pretty(rx_message_).c_str());
    return false;
  }

  // Calculate CRC2 (bytes 10-end-1)
  uint8_t crc2 = rx_message_[10];
  for (uint8_t i = 11; i < expected_length - 1; i++) {
    crc2 ^= rx_message_[i];
  }

  if (rx_message_[expected_length - 1] != crc2) {
    ESP_LOGW(TAG, "CRC2 error: calc=%02X recv=%02X", crc2, rx_message_[expected_length - 1]);
    return false;
  }

  // Process valid message
  rx_message_.erase(rx_message_.begin());
  parse_status_packet(rx_message_);
  return false;
}

void NiceBusT4::send_array_cmd(const uint8_t *data, size_t len) {
  // Send BREAK condition
  uart_set_baudrate(_uart, BAUD_BREAK);
  uart_write(_uart, "\x00", 1);
  uart_flush(_uart);
  delayMicroseconds(1000);
  
  // Send actual data
  uart_set_baudrate(_uart, BAUD_WORK);
  delayMicroseconds(200);
  uart_write(_uart, (char *)data, len);
  uart_flush(_uart);
  
  ESP_LOGD(TAG, "Sent: %s", format_hex_pretty(data, len).c_str());
}

std::vector<uint8_t> NiceBusT4::gen_control_cmd(const uint8_t control_cmd) {
  std::vector<uint8_t> frame = {addr_to[0], addr_to[1], addr_from[0], addr_from[1]};
  frame.push_back(CMD);
  frame.push_back(0x05);
  
  uint8_t crc1 = frame[0];
  for (size_t i = 1; i < frame.size(); i++) {
    crc1 ^= frame[i];
  }
  frame.push_back(crc1);
  
  frame.push_back(CONTROL);
  frame.push_back(RUN);
  frame.push_back(control_cmd);
  frame.push_back(0x64); // Offset
  
  uint8_t crc2 = frame[7];
  for (size_t i = 8; i < frame.size(); i++) {
    crc2 ^= frame[i];
  }
  frame.push_back(crc2);
  
  uint8_t f_size = frame.size();
  frame.push_back(f_size);
  frame.insert(frame.begin(), f_size);
  frame.insert(frame.begin(), START_CODE);
  
  return frame;
}

std::vector<uint8_t> NiceBusT4::gen_inf_cmd(const uint8_t to_addr1, const uint8_t to_addr2,
                                          const uint8_t whose, const uint8_t inf_cmd,
                                          const uint8_t run_cmd, const uint8_t next_data,
                                          const std::vector<uint8_t> &data, size_t len) {
  std::vector<uint8_t> frame = {to_addr1, to_addr2, addr_from[0], addr_from[1]};
  frame.push_back(INF);
  frame.push_back(0x06 + len);
  
  uint8_t crc1 = frame[0];
  for (size_t i = 1; i < frame.size(); i++) {
    crc1 ^= frame[i];
  }
  frame.push_back(crc1);
  
  frame.push_back(whose);
  frame.push_back(inf_cmd);
  frame.push_back(run_cmd);
  frame.push_back(next_data);
  frame.push_back(len);
  
  if (len > 0) {
    frame.insert(frame.end(), data.begin(), data.end());
  }
  
  uint8_t crc2 = frame[7];
  for (size_t i = 8; i < frame.size(); i++) {
    crc2 ^= frame[i];
  }
  frame.push_back(crc2);
  
  uint8_t f_size = frame.size();
  frame.push_back(f_size);
  frame.insert(frame.begin(), f_size);
  frame.insert(frame.begin(), START_CODE);
  
  return frame;
}

void NiceBusT4::parse_status_packet(const std::vector<uint8_t> &data) {
  // [Реализация разбора пакетов остается без изменений]
  // ...
}

void NiceBusT4::dump_config() {
  // [Реализация вывода конфигурации остается без изменений]
  // ...
}

}  // namespace bus_t4
}  // namespace esphome
