#include "nice-bust4.h"
#include "esphome/core/log.h"
#include "esphome/core/helpers.h"
#include <algorithm>

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

  // After initialization, request position limits
  if (init_ok && !limits_loaded) {
    tx_buffer_.push(gen_inf_cmd(addr_to[0], addr_to[1], FOR_CU, POS_MAX, GET, 0x00));
    tx_buffer_.push(gen_inf_cmd(addr_to[0], addr_to[1], FOR_CU, POS_MIN, GET, 0x00));
    limits_loaded = true;
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
    return;
  }

  if (call.get_position().has_value()) {
    float newpos = *call.get_position();
    
    if (newpos == COVER_OPEN) {
      send_cmd(OPEN);
    } else if (newpos == COVER_CLOSED) {
      send_cmd(CLOSE);
    } else {
      // Use predefined partial openings
      if (newpos <= 0.25f) {
        send_cmd(P_OPN1);
      } else if (newpos <= 0.50f) {
        send_cmd(P_OPN2);
      } else if (newpos <= 0.75f) {
        send_cmd(P_OPN3);
      } else {
        send_cmd(P_OPN4);
      }
    }
  }
}

// Остальные функции остаются как в предыдущем исправлении
// validate_message_, send_array_cmd, gen_control_cmd, gen_inf_cmd, 
// parse_status_packet, update_position, publish_state_if_changed

void NiceBusT4::parse_status_packet(const std::vector<uint8_t> &data) {
  // Определение крайних положений
  if (data[10] == POS_MAX) {
    _pos_opn = (data[14] << 8) + data[15];
    ESP_LOGI(TAG, "Open position detected: %d", _pos_opn);
  }
  
  if (data[10] == POS_MIN) {
    _pos_cls = (data[14] << 8) + data[15];
    ESP_LOGI(TAG, "Closed position detected: %d", _pos_cls);
  }
  
  // Обработка статуса
  if (data[10] == INF_STATUS) {
    switch (data[14]) {
      case OPENED:
        position = 100.0f;
        current_operation = COVER_OPERATION_IDLE;
        break;
      case CLOSED:
        position = 0.0f;
        current_operation = COVER_OPERATION_IDLE;
        break;
      case STA_OPENING:
        current_operation = COVER_OPERATION_OPENING;
        // Рассчитываем процент открытия
        if (_pos_opn != _pos_cls) {
          position = 100.0f * (_pos_usl - _pos_cls) / (_pos_opn - _pos_cls);
        }
        break;
      case STA_CLOSING:
        current_operation = COVER_OPERATION_CLOSING;
        if (_pos_opn != _pos_cls) {
          position = 100.0f * (_pos_usl - _pos_cls) / (_pos_opn - _pos_cls);
        }
        break;
    }
    publish_state_if_changed();
  }
}

void NiceBusT4::update_position(uint16_t newpos) {
  _pos_usl = newpos;
  
  // Рассчитываем процент открытия
  if (_pos_opn != _pos_cls) {
    position = 100.0f * (static_cast<float>(newpos - _pos_cls) / 
                        static_cast<float>(_pos_opn - _pos_cls);
    position = std::clamp(position, 0.0f, 100.0f);
  }

  ESP_LOGD(TAG, "Position update: raw=%d, calc=%.1f%%", newpos, position);
  publish_state_if_changed();
}

}  // namespace bus_t4
}  // namespace esphome
