#include "nice-bust4.h"
#include "esphome/core/log.h"
#include "esphome/core/helpers.h"
#include <algorithm>
#include <deque>

namespace esphome {
namespace bus_t4 {

static const char *TAG = "bus_t4.cover";

// Конфигурация фильтра
constexpr size_t POSITION_FILTER_SIZE = 5;
constexpr uint16_t POSITION_THRESHOLD = 50;  // Минимальное изменение для обновления

using namespace esphome::cover;

CoverTraits NiceBusT4::get_traits() {
  auto traits = CoverTraits();
  traits.set_supports_position(true);
  traits.set_supports_stop(true);
  return traits;
}

void NiceBusT4::setup() {
  _uart = uart_init(_UART_NO, BAUD_WORK, SERIAL_8N1, SERIAL_FULL, TX_P, 512, true);
  
  // Сброс линии
  uart_set_baudrate(_uart, 300);
  uart_write(_uart, "\x00\x00\x00", 3);
  uart_flush(_uart);
  delay(100);
  uart_set_baudrate(_uart, BAUD_WORK);
  
  // Инициализация
  this->tx_buffer_.push(gen_inf_cmd(0x00, 0xFF, FOR_ALL, WHO, GET, 0x00));
}

void NiceBusT4::loop() {
  uint32_t now = millis();

  // Обновление при инициализации
  if (!init_ok && (now - last_update_) > 10000) {
    this->tx_buffer_.push(gen_inf_cmd(0x00, 0xFF, FOR_ALL, WHO, GET, 0x00));
    last_update_ = now;
  }

  // Запрос крайних положений
  if (init_ok && !limits_loaded) {
    tx_buffer_.push(gen_inf_cmd(addr_to[0], addr_to[1], FOR_CU, POS_MAX, GET, 0x00));
    tx_buffer_.push(gen_inf_cmd(addr_to[0], addr_to[1], FOR_CU, POS_MIN, GET, 0x00));
    limits_loaded = true;
  }

  // Обработка входящих данных
  while (uart_rx_available(_uart)) {
    uint8_t c = uart_read_char(_uart);
    this->handle_char_(c);
    last_uart_byte_ = now;
  }

  // Отправка команд
  if (ready_to_tx_ && !tx_buffer_.empty() && (now - last_uart_byte_ > 100)) {
    send_array_cmd(tx_buffer_.front());
    tx_buffer_.pop();
    ready_to_tx_ = false;
    last_uart_byte_ = now;
  }

  // Запрос позиции с регулируемым интервалом
  if (init_ok && (now - last_position_time > position_update_interval)) {
    if (current_operation != COVER_OPERATION_IDLE) {
      request_position();
      last_position_time = now;
      
      // Увеличиваем интервал при движении
      position_update_interval = 300; // 300ms при движении
    } else {
      // Редкие обновления в покое
      if (now - last_position_time > 5000) {
        request_position();
        last_position_time = now;
        position_update_interval = 100; // Сброс интервала
      }
    }
  }
}

void NiceBusT4::update_position(uint16_t newpos) {
  // Фильтрация выбросов
  static std::deque<uint16_t> position_history;
  position_history.push_back(newpos);
  
  if (position_history.size() > POSITION_FILTER_SIZE) {
    position_history.pop_front();
  }
  
  // Медианный фильтр
  std::vector<uint16_t> sorted(position_history.begin(), position_history.end());
  std::sort(sorted.begin(), sorted.end());
  uint16_t filtered_pos = sorted[sorted.size() / 2];
  
  // Рассчет положения
  if (_pos_opn > _pos_cls && _pos_opn != 0) {
    float new_position = 100.0f * (static_cast<float>(filtered_pos - _pos_cls) / 
                                  static_cast<float>(_pos_opn - _pos_cls);
    
    new_position = std::clamp(new_position, 0.0f, 100.0f);
    
    // Фильтр низких частот
    position = 0.7f * position + 0.3f * new_position;
    
    // Обновляем только при значительном изменении
    if (fabs(position - last_published_pos) > 2.0f) {
      publish_state();
      last_published_pos = position;
    }
  }

  ESP_LOGD(TAG, "Position: raw=%d, filtered=%d, calc=%.1f%%", 
          newpos, filtered_pos, position);
}

void NiceBusT4::parse_status_packet(const std::vector<uint8_t> &data) {
  // Определение крайних положений
  if (data[10] == POS_MAX) {
    uint16_t new_opn = (data[14] << 8) + data[15];
    if (new_opn > 0 && new_opn != _pos_opn) {
      _pos_opn = new_opn;
      ESP_LOGI(TAG, "Open position updated: %d", _pos_opn);
    }
  }
  
  if (data[10] == POS_MIN) {
    uint16_t new_cls = (data[14] << 8) + data[15];
    if (new_cls != _pos_cls) {
      _pos_cls = new_cls;
      ESP_LOGI(TAG, "Closed position updated: %d", _pos_cls);
    }
  }
  
  // Обработка статуса движения
  if (data[10] == INF_STATUS || (data[1] > 0x0d && data[9] == FOR_CU)) {
    CoverOperation new_operation = current_operation;
    
    switch (data[11]) {
      case STA_OPENING:
        new_operation = COVER_OPERATION_OPENING;
        break;
      case STA_CLOSING:
        new_operation = COVER_OPERATION_CLOSING;
        break;
      case OPENED:
      case CLOSED:
      case STOPPED:
        new_operation = COVER_OPERATION_IDLE;
        break;
    }
    
    // Обновляем статус только при изменении
    if (new_operation != current_operation) {
      current_operation = new_operation;
      publish_state();
      
      // При изменении статуса сразу запрашиваем позицию
      if (current_operation != COVER_OPERATION_IDLE) {
        request_position();
        last_position_time = millis();
      }
    }
  }
}

// Остальные функции остаются без существенных изменений
// ...
}  // namespace bus_t4
}  // namespace esp_home
