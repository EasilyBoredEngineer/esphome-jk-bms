#include "heltec_balancer_ble.h"
#include "esphome/core/log.h"

#ifdef USE_ESP32

namespace esphome {
namespace heltec_balancer_ble {

static const char *const TAG = "heltec_balancer_ble";

static constexpr uint8_t MAX_NO_RESPONSE_COUNT = 10;
static constexpr uint16_t HELTEC_BALANCER_SERVICE_UUID = 0xFFE0;
static constexpr uint16_t HELTEC_BALANCER_CHARACTERISTIC_UUID = 0xFFE1;

static constexpr uint8_t SOF_REQUEST_BYTE1 = 0xAA;
static constexpr uint8_t SOF_REQUEST_BYTE2 = 0x55;
static constexpr uint8_t SOF_RESPONSE_BYTE1 = 0x55;
static constexpr uint8_t SOF_RESPONSE_BYTE2 = 0xAA;
static constexpr uint8_t DEVICE_ADDRESS = 0x11;

static constexpr uint8_t FUNCTION_WRITE = 0x00;
static constexpr uint8_t FUNCTION_READ = 0x01;

static constexpr uint8_t COMMAND_NONE = 0x00;
static constexpr uint8_t COMMAND_DEVICE_INFO = 0x01;
static constexpr uint8_t COMMAND_CELL_INFO = 0x02;
static constexpr uint8_t COMMAND_FACTORY_DEFAULTS = 0x03;
static constexpr uint8_t COMMAND_SETTINGS = 0x04;
static constexpr uint8_t COMMAND_WRITE_REGISTER = 0x05;

static constexpr uint8_t END_OF_FRAME = 0xFF;

static constexpr uint16_t MIN_RESPONSE_SIZE = 20;
static constexpr uint16_t MAX_RESPONSE_SIZE = 300;

static constexpr uint8_t OPERATION_STATUS_SIZE = 13;
static constexpr uint8_t BUZZER_MODES_SIZE = 4;
static constexpr uint8_t BATTERY_TYPES_SIZE = 5;
static constexpr uint8_t CELL_ERRORS_SIZE = 8;

// Simple string arrays - no PROGMEM
static const char *const OPERATION_STATUS[] = {
    "Unknown",
    "Wrong cell count",
    "AcqLine Res test",
    "AcqLine Res exceed",
    "Systest Completed",
    "Balancing",
    "Balancing finished",
    "Low voltage",
    "System Overtemp",
    "Host fails",
    "Low battery voltage - balancing stopped",
    "Temperature too high - balancing stopped",
    "Self-test completed"
};

static const char *const BUZZER_MODES[] = {
    "Unknown",
    "Off",
    "Beep once",
    "Beep regular"
};

static const char *const BATTERY_TYPES[] = {
    "Unknown",
    "NCM",
    "LFP",
    "LTO",
    "PbAc"
};

static const char *const CELL_ERRORS[] = {
    "Battery detection failed",
    "Overvoltage",
    "Undervoltage",
    "Polarity error",
    "Excessive line resistance",
    "System overheating",
    "Charging fault",
    "Discharge fault"
};

uint8_t crc(const uint8_t data[], const uint16_t len) {
  uint8_t crc = 0;
  for (uint16_t i = 0; i < len; i++) {
    crc = crc + data[i];
  }
  return crc;
}

void HeltecBalancerBle::dump_config() {
  ESP_LOGCONFIG(TAG, "HeltecBalancerBle");
  LOG_BINARY_SENSOR("", "Balancing", this->balancing_binary_sensor_);
  LOG_BINARY_SENSOR("", "Online Status", this->online_status_binary_sensor_);
  LOG_SENSOR("", "Minimum Cell Voltage", this->min_cell_voltage_sensor_);
  LOG_SENSOR("", "Maximum Cell Voltage", this->max_cell_voltage_sensor_);
  LOG_SENSOR("", "Delta Cell Voltage", this->delta_cell_voltage_sensor_);
  LOG_SENSOR("", "Average Cell Voltage", this->average_cell_voltage_sensor_);
  LOG_SENSOR("", "Total Voltage", this->total_voltage_sensor_);
  LOG_TEXT_SENSOR("", "Operation Status", this->operation_status_text_sensor_);
}

void HeltecBalancerBle::gattc_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if,
                                            esp_ble_gattc_cb_param_t *param) {
  switch (event) {
    case ESP_GATTC_OPEN_EVT: {
      break;
    }
    case ESP_GATTC_DISCONNECT_EVT: {
      ESP_LOGI(TAG, "Disconnect event - cleaning up");
      
      // CRITICAL: Save handle before clearing state
      uint16_t handle_to_unregister = this->char_handle_;
      
      // CRITICAL: Unregister notifications BEFORE clearing state
      if (handle_to_unregister != 0) {
        auto status = esp_ble_gattc_unregister_for_notify(this->parent()->get_gattc_if(),
                                                          this->parent()->get_remote_bda(), handle_to_unregister);
        if (status) {
          ESP_LOGW(TAG, "esp_ble_gattc_unregister_for_notify failed, status=%d", status);
        }
      }
      
      // CRITICAL: Set state first to prevent race conditions
      this->node_state = espbt::ClientState::IDLE;
      this->status_notification_received_ = false;
      this->char_handle_ = 0;
      
      // CRITICAL: Clear buffer LAST to avoid accessing during notifications
      this->frame_buffer_.clear();
      this->frame_buffer_.shrink_to_fit();

      break;
    }
    case ESP_GATTC_SEARCH_CMPL_EVT: {
      auto *chr = this->parent_->get_characteristic(HELTEC_BALANCER_SERVICE_UUID, HELTEC_BALANCER_CHARACTERISTIC_UUID);
      if (chr == nullptr) {
        ESP_LOGE(TAG, "[%s] No control service found at device, not an Heltec/NEEY balancer..?",
                 this->parent_->address_str().c_str());
        break;
      }

      this->char_handle_ = chr->handle;
      
      // Pre-allocate buffer to avoid reallocations during operation
      this->frame_buffer_.clear();
      this->frame_buffer_.reserve(MAX_RESPONSE_SIZE);

      auto status = esp_ble_gattc_register_for_notify(this->parent()->get_gattc_if(), this->parent()->get_remote_bda(),
                                                      chr->handle);
      if (status) {
        ESP_LOGW(TAG, "esp_ble_gattc_register_for_notify failed, status=%d", status);
      }
      break;
    }
    case ESP_GATTC_REG_FOR_NOTIFY_EVT: {
      this->node_state = espbt::ClientState::ESTABLISHED;
      this->status_notification_received_ = false;

      ESP_LOGI(TAG, "Request device info");
      this->send_command(FUNCTION_READ, COMMAND_DEVICE_INFO);

      break;
    }
    case ESP_GATTC_NOTIFY_EVT: {
      // CRITICAL: Check handle is still valid
      if (this->char_handle_ == 0 || param->notify.handle != this->char_handle_)
        break;

      ESP_LOGVV(TAG, "Notification received: %s",
                format_hex_pretty(param->notify.value, param->notify.value_len).c_str());

      this->assemble(param->notify.value, param->notify.value_len);

      break;
    }
    default:
      break;
  }
}

void HeltecBalancerBle::update() {
  this->track_online_status_();
  if (this->node_state != espbt::ClientState::ESTABLISHED) {
    ESP_LOGW(TAG, "[%s] Not connected", this->parent_->address_str().c_str());
    return;
  }

  if (!this->status_notification_received_) {
    ESP_LOGI(TAG, "Request status notification");
    this->send_command(FUNCTION_READ, COMMAND_CELL_INFO);
  }
}

void HeltecBalancerBle::assemble(const uint8_t *data, uint16_t length) {
  // CRITICAL: Check if still connected to avoid processing stale data
  if (this->node_state != espbt::ClientState::ESTABLISHED || this->char_handle_ == 0) {
    ESP_LOGV(TAG, "Ignoring data - not in established state");
    return;
  }

  // Safety check for buffer overflow
  if (this->frame_buffer_.size() > MAX_RESPONSE_SIZE) {
    ESP_LOGW(TAG, "Frame dropped because of invalid length");
    this->frame_buffer_.clear();
    return;
  }

  // Flush buffer on every preamble
  if (length >= 2 && data[0] == SOF_RESPONSE_BYTE1 && data[1] == SOF_RESPONSE_BYTE2) {
    this->frame_buffer_.clear();
  }

  this->frame_buffer_.insert(this->frame_buffer_.end(), data, data + length);

  if (this->frame_buffer_.size() >= MIN_RESPONSE_SIZE && this->frame_buffer_.back() == END_OF_FRAME) {
    const uint8_t *raw = &this->frame_buffer_[0];
    const uint16_t frame_size = this->frame_buffer_.size();

    uint8_t computed_crc = crc(raw, frame_size - 2);
    uint8_t remote_crc = raw[frame_size - 2];
    if (computed_crc != remote_crc) {
      ESP_LOGW(TAG, "CRC check failed! 0x%02X != 0x%02X", computed_crc, remote_crc);
      this->frame_buffer_.clear();
      return;
    }

    // Pass buffer directly instead of copying
    std::vector<uint8_t> data(this->frame_buffer_.begin(), this->frame_buffer_.end());

    this->decode_(data);
    this->frame_buffer_.clear();
  }
}

void HeltecBalancerBle::decode_(const std::vector<uint8_t> &data) {
  this->reset_online_status_tracker_();

  uint8_t frame_type = data[4];
  switch (frame_type) {
    case COMMAND_DEVICE_INFO:
      this->decode_device_info_(data);
      break;
    case COMMAND_CELL_INFO:
      this->decode_cell_info_(data);
      break;
    case COMMAND_FACTORY_DEFAULTS:
      this->decode_factory_defaults_(data);
      break;
    case COMMAND_SETTINGS:
      this->decode_settings_(data);
      break;
    case COMMAND_WRITE_REGISTER:
      ESP_LOGD(TAG, "Write register response received: %s", format_hex_pretty(data.data(), data.size()).c_str());
      break;
    default:
      ESP_LOGW(TAG, "Unsupported message type (0x%02X)", data[4]);
  }
}

void HeltecBalancerBle::decode_cell_info_(const std::vector<uint8_t> &data) {
  auto heltec_get_16bit = [&](size_t i) -> uint16_t {
    return (uint16_t(data[i + 1]) << 8) | (uint16_t(data[i + 0]) << 0);
  };
  auto heltec_get_24bit = [&](size_t i) -> uint32_t {
    return (uint32_t(data[i + 2]) << 16) | (uint32_t(data[i + 1]) << 8) | (uint32_t(data[i + 0]) << 0);
  };
  auto heltec_get_32bit = [&](size_t i) -> uint32_t {
    return (uint32_t(heltec_get_16bit(i + 2)) << 16) | (uint32_t(heltec_get_16bit(i + 0)) << 0);
  };

  const uint32_t now = millis();
  if (now - this->last_cell_info_ < this->throttle_) {
    return;
  }
  this->last_cell_info_ = now;

  ESP_LOGI(TAG, "Cell info frame (%d bytes):", data.size());
  ESP_LOGD(TAG, "  %s", format_hex_pretty(&data.front(), 150).c_str());
  ESP_LOGD(TAG, "  %s", format_hex_pretty(&data.front() + 150, data.size() - 150).c_str());

  ESP_LOGD(TAG, "  Frame counter: %d", data[8]);

  // Calculate cell statistics in a single pass - optimized
  uint8_t cells = 24;
  uint8_t cells_enabled = 0;
  float min_cell_voltage = 100.0f;
  float max_cell_voltage = -100.0f;
  float average_cell_voltage = 0.0f;
  uint8_t min_voltage_cell = 0;
  uint8_t max_voltage_cell = 0;
  
  for (uint8_t i = 0; i < cells; i++) {
    float cell_voltage = ieee_float_(heltec_get_32bit(i * 4 + 9));
    float cell_resistance = ieee_float_(heltec_get_32bit(i * 4 + 105));
    
    if (cell_voltage > 0) {
      average_cell_voltage = average_cell_voltage + cell_voltage;
      cells_enabled++;
      
      if (cell_voltage < min_cell_voltage) {
        min_cell_voltage = cell_voltage;
        min_voltage_cell = i + 1;
      }
    }
    if (cell_voltage > max_cell_voltage) {
      max_cell_voltage = cell_voltage;
      max_voltage_cell = i + 1;
    }
    
    this->publish_state_(this->cells_[i].cell_voltage_sensor_, cell_voltage);
    this->publish_state_(this->cells_[i].cell_resistance_sensor_, cell_resistance);
  }
  
  if (cells_enabled > 0) {
    average_cell_voltage = average_cell_voltage / cells_enabled;
  }

  this->publish_state_(this->min_cell_voltage_sensor_, min_cell_voltage);
  this->publish_state_(this->max_cell_voltage_sensor_, max_cell_voltage);
  this->publish_state_(this->min_voltage_cell_sensor_, (float) min_voltage_cell);
  this->publish_state_(this->max_voltage_cell_sensor_, (float) max_voltage_cell);
  this->publish_state_(this->delta_cell_voltage_sensor_, max_cell_voltage - min_cell_voltage);
  this->publish_state_(this->average_cell_voltage_sensor_, average_cell_voltage);

  this->publish_state_(this->total_voltage_sensor_, ieee_float_(heltec_get_32bit(201)));

  uint8_t raw_operation_status = data[216];
  this->publish_state_(this->balancing_binary_sensor_, (raw_operation_status == 0x05));
  if (raw_operation_status < OPERATION_STATUS_SIZE) {
    this->publish_state_(this->operation_status_text_sensor_, OPERATION_STATUS[raw_operation_status]);
  } else {
    this->publish_state_(this->operation_status_text_sensor_, "Unknown");
  }

  this->publish_state_(this->balancing_current_sensor_, ieee_float_(heltec_get_32bit(217)));
  this->publish_state_(this->temperature_sensor_1_sensor_, ieee_float_(heltec_get_32bit(221)));
  this->publish_state_(this->temperature_sensor_2_sensor_, ieee_float_(heltec_get_32bit(225)));
  this->publish_state_(this->cell_detection_failed_bitmask_sensor_, heltec_get_24bit(229));
  this->publish_state_(this->cell_overvoltage_bitmask_sensor_, heltec_get_24bit(232));
  this->publish_state_(this->cell_undervoltage_bitmask_sensor_, heltec_get_24bit(235));
  this->publish_state_(this->cell_polarity_error_bitmask_sensor_, heltec_get_24bit(238));
  this->publish_state_(this->cell_excessive_line_resistance_bitmask_sensor_, heltec_get_24bit(241));
  this->publish_state_(this->error_system_overheating_binary_sensor_, data[244] != 0x00);
  this->publish_state_(this->error_charging_binary_sensor_, (bool) data[245]);
  this->publish_state_(this->error_discharging_binary_sensor_, (bool) data[246]);

  ESP_LOGI(TAG, "  Uptime: %s (%lus)", format_total_runtime_(heltec_get_32bit(254)).c_str(),
           (unsigned long) heltec_get_32bit(254));

  this->status_notification_received_ = true;
}

void HeltecBalancerBle::decode_settings_(const std::vector<uint8_t> &data) {
  auto heltec_get_16bit = [&](size_t i) -> uint16_t {
    return (uint16_t(data[i + 1]) << 8) | (uint16_t(data[i + 0]) << 0);
  };
  auto heltec_get_32bit = [&](size_t i) -> uint32_t {
    return (uint32_t(heltec_get_16bit(i + 2)) << 16) | (uint32_t(heltec_get_16bit(i + 0)) << 0);
  };

  ESP_LOGI(TAG, "Settings frame (%d bytes):", data.size());
  ESP_LOGD(TAG, "  %s", format_hex_pretty(data.data(), data.size()).c_str());

  this->publish_state_(this->cell_count_number_, (float) data[8]);
  this->publish_state_(this->balance_trigger_voltage_number_, ieee_float_(heltec_get_32bit(9)));
  this->publish_state_(this->max_balance_current_number_, ieee_float_(heltec_get_32bit(13)));
  this->publish_state_(this->balance_sleep_voltage_number_, ieee_float_(heltec_get_32bit(17)));

  uint8_t raw_balancer_enabled = data[21];
  this->publish_state_(this->balancer_switch_, (bool) raw_balancer_enabled);

  uint8_t raw_buzzer_mode = data[22];
  if (raw_buzzer_mode < BUZZER_MODES_SIZE) {
    this->publish_state_(this->buzzer_mode_text_sensor_, BUZZER_MODES[raw_buzzer_mode]);
  } else {
    this->publish_state_(this->buzzer_mode_text_sensor_, "Unknown");
  }

  uint8_t raw_battery_type = data[23];
  if (raw_battery_type < BATTERY_TYPES_SIZE) {
    this->publish_state_(this->battery_type_text_sensor_, BATTERY_TYPES[raw_battery_type]);
  } else {
    this->publish_state_(this->battery_type_text_sensor_, "Unknown");
  }

  this->publish_state_(this->nominal_battery_capacity_number_, (float) heltec_get_32bit(24));
  this->publish_state_(this->balance_start_voltage_number_, ieee_float_(heltec_get_32bit(28)));
}

void HeltecBalancerBle::decode_factory_defaults_(const std::vector<uint8_t> &data) {
  auto heltec_get_16bit = [&](size_t i) -> uint16_t {
    return (uint16_t(data[i + 1]) << 8) | (uint16_t(data[i + 0]) << 0);
  };
  auto heltec_get_32bit = [&](size_t i) -> uint32_t {
    return (uint32_t(heltec_get_16bit(i + 2)) << 16) | (uint32_t(heltec_get_16bit(i + 0)) << 0);
  };

  ESP_LOGI(TAG, "Factory defaults frame (%d bytes):", data.size());
  ESP_LOGD(TAG, "  %s", format_hex_pretty(data.data(), data.size()).c_str());

  // Skip the acknowledge frame
  if (data.size() == 20) {
    return;
  }

  ESP_LOGI(TAG, "  Standard voltage 1: %.3f V", ieee_float_(heltec_get_32bit(8)));
  ESP_LOGI(TAG, "  Standard voltage 2: %.3f V", ieee_float_(heltec_get_32bit(12)));
  ESP_LOGI(TAG, "  Battery voltage 1: %.3f V", ieee_float_(heltec_get_32bit(16)));
  ESP_LOGI(TAG, "  Battery voltage 2: %.3f V", ieee_float_(heltec_get_32bit(20)));
  ESP_LOGI(TAG, "  Standard current 1: %.3f A", ieee_float_(heltec_get_32bit(24)));
  ESP_LOGI(TAG, "  Standard current 2: %.3f A", ieee_float_(heltec_get_32bit(28)));
  ESP_LOGI(TAG, "  SuperBat 1: %.3f", ieee_float_(heltec_get_32bit(32)));
  ESP_LOGI(TAG, "  SuperBat 2: %.3f", ieee_float_(heltec_get_32bit(36)));
  ESP_LOGI(TAG, "  Resistor 1: %.3f", ieee_float_(heltec_get_32bit(40)));
  ESP_LOGI(TAG, "  Battery status: %.3f", ieee_float_(heltec_get_32bit(44)));
  ESP_LOGI(TAG, "  Max voltage: %.3f V", ieee_float_(heltec_get_32bit(48)));
  ESP_LOGI(TAG, "  Min voltage: %.3f V", ieee_float_(heltec_get_32bit(52)));
  ESP_LOGI(TAG, "  Max temperature: %.3f °C", ieee_float_(heltec_get_32bit(56)));
  ESP_LOGI(TAG, "  Min temperature: %.3f °C", ieee_float_(heltec_get_32bit(60)));
  ESP_LOGI(TAG, "  Power on counter: %lu", (unsigned long) heltec_get_32bit(64));
  ESP_LOGI(TAG, "  Total runtime: %lu", (unsigned long) heltec_get_32bit(68));
  ESP_LOGI(TAG, "  Production date: %s", std::string(data.begin() + 72, data.begin() + 72 + 8).c_str());
}

void HeltecBalancerBle::decode_device_info_(const std::vector<uint8_t> &data) {
  auto heltec_get_16bit = [&](size_t i) -> uint16_t {
    return (uint16_t(data[i + 1]) << 8) | (uint16_t(data[i + 0]) << 0);
  };
  auto heltec_get_32bit = [&](size_t i) -> uint32_t {
    return (uint32_t(heltec_get_16bit(i + 2)) << 16) | (uint32_t(heltec_get_16bit(i + 0)) << 0);
  };

  ESP_LOGI(TAG, "Device info frame (%d bytes):", data.size());
  ESP_LOGD(TAG, "  %s", format_hex_pretty(data.data(), data.size()).c_str());

  ESP_LOGI(TAG, "  Model: %s", std::string(data.begin() + 8, data.begin() + 8 + 16).c_str());
  ESP_LOGI(TAG, "  Hardware version: %s", std::string(data.begin() + 24, data.begin() + 24 + 8).c_str());
  ESP_LOGI(TAG, "  Software version: %s", std::string(data.begin() + 32, data.begin() + 32 + 8).c_str());
  ESP_LOGI(TAG, "  Protocol version: %s", std::string(data.begin() + 40, data.begin() + 40 + 8).c_str());
  ESP_LOGI(TAG, "  Manufacturing date: %s", std::string(data.begin() + 48, data.begin() + 48 + 8).c_str());
  ESP_LOGI(TAG, "  Power on count: %d", heltec_get_16bit(56));
  ESP_LOGI(TAG, "  Total runtime: %s (%lus)", format_total_runtime_(heltec_get_32bit(60)).c_str(),
           (unsigned long) heltec_get_32bit(60));
  this->publish_state_(this->total_runtime_sensor_, (float) heltec_get_32bit(60));
  this->publish_state_(this->total_runtime_formatted_text_sensor_, format_total_runtime_(heltec_get_32bit(60)));
}

bool HeltecBalancerBle::send_command(uint8_t function, uint8_t command, uint8_t register_address, uint32_t value) {
  // CRITICAL: Check connection state before sending
  if (this->node_state != espbt::ClientState::ESTABLISHED || this->char_handle_ == 0) {
    ESP_LOGW(TAG, "Cannot send command - not connected");
    return false;
  }

  uint16_t length = 0x0014;

  uint8_t frame[20];
  frame[0] = SOF_REQUEST_BYTE1;
  frame[1] = SOF_REQUEST_BYTE2;
  frame[2] = DEVICE_ADDRESS;
  frame[3] = function;
  frame[4] = command >> 0;
  frame[5] = register_address;
  frame[6] = length >> 0;
  frame[7] = length >> 8;
  frame[8] = value >> 0;
  frame[9] = value >> 8;
  frame[10] = value >> 16;
  frame[11] = value >> 24;
  frame[12] = 0x00;
  frame[13] = 0x00;
  frame[14] = 0x00;
  frame[15] = 0x00;
  frame[16] = 0x00;
  frame[17] = 0x00;
  frame[18] = crc(frame, sizeof(frame) - 2);
  frame[19] = END_OF_FRAME;

  ESP_LOGD(TAG, "Write register: %s", format_hex_pretty(frame, sizeof(frame)).c_str());
  auto status =
      esp_ble_gattc_write_char(this->parent_->get_gattc_if(), this->parent_->get_conn_id(), this->char_handle_,
                               sizeof(frame), frame, ESP_GATT_WRITE_TYPE_NO_RSP, ESP_GATT_AUTH_REQ_NONE);

  if (status) {
    ESP_LOGW(TAG, "[%s] esp_ble_gattc_write_char failed, status=%d", this->parent_->address_str().c_str(), status);
  }

  return (status == 0);
}

void HeltecBalancerBle::track_online_status_() {
  if (this->no_response_count_ < MAX_NO_RESPONSE_COUNT) {
    this->no_response_count_++;
  }
  if (this->no_response_count_ == MAX_NO_RESPONSE_COUNT) {
    this->publish_device_unavailable_();
    this->no_response_count_++;
  }
}

void HeltecBalancerBle::reset_online_status_tracker_() {
  this->no_response_count_ = 0;
  this->publish_state_(this->online_status_binary_sensor_, true);
}

void HeltecBalancerBle::publish_device_unavailable_() {
  this->publish_state_(this->online_status_binary_sensor_, false);
  this->publish_state_(this->operation_status_text_sensor_, "Offline");

  this->publish_state_(min_cell_voltage_sensor_, NAN);
  this->publish_state_(max_cell_voltage_sensor_, NAN);
  this->publish_state_(min_voltage_cell_sensor_, NAN);
  this->publish_state_(max_voltage_cell_sensor_, NAN);
  this->publish_state_(delta_cell_voltage_sensor_, NAN);
  this->publish_state_(average_cell_voltage_sensor_, NAN);
  this->publish_state_(total_voltage_sensor_, NAN);
  this->publish_state_(temperature_sensor_1_sensor_, NAN);
  this->publish_state_(temperature_sensor_2_sensor_, NAN);
  this->publish_state_(total_runtime_sensor_, NAN);
  this->publish_state_(balancing_current_sensor_, NAN);
  this->publish_state_(errors_bitmask_sensor_, NAN);
  this->publish_state_(cell_detection_failed_bitmask_sensor_, NAN);
  this->publish_state_(cell_overvoltage_bitmask_sensor_, NAN);
  this->publish_state_(cell_undervoltage_bitmask_sensor_, NAN);
  this->publish_state_(cell_polarity_error_bitmask_sensor_, NAN);
  this->publish_state_(cell_excessive_line_resistance_bitmask_sensor_, NAN);

  // CRITICAL BUG FIX: Also publish NAN for cell resistance sensors
  for (auto &cell : this->cells_) {
    this->publish_state_(cell.cell_voltage_sensor_, NAN);
    this->publish_state_(cell.cell_resistance_sensor_, NAN);
  }
}

void HeltecBalancerBle::publish_state_(binary_sensor::BinarySensor *binary_sensor, const bool &state) {
  if (binary_sensor == nullptr)
    return;
  binary_sensor->publish_state(state);
}

void HeltecBalancerBle::publish_state_(number::Number *number, float value) {
  if (number == nullptr)
    return;
  number->publish_state(value);
}

void HeltecBalancerBle::publish_state_(sensor::Sensor *sensor, float value) {
  if (sensor == nullptr)
    return;
  sensor->publish_state(value);
}

void HeltecBalancerBle::publish_state_(switch_::Switch *obj, const bool &state) {
  if (obj == nullptr)
    return;
  obj->publish_state(state);
}

void HeltecBalancerBle::publish_state_(text_sensor::TextSensor *text_sensor, const std::string &state) {
  if (text_sensor == nullptr)
    return;
  text_sensor->publish_state(state);
}

std::string HeltecBalancerBle::error_bits_to_string_(uint16_t bitmask) {
  std::string errors_list = "";
  if (bitmask) {
    for (uint8_t i = 0; i < CELL_ERRORS_SIZE; i++) {
      if (bitmask & (1 << i)) {
        errors_list.append(CELL_ERRORS[i]);
        errors_list.append(";");
      }
    }
    if (!errors_list.empty()) {
      errors_list.pop_back();
    }
  }
  return errors_list;
}

}  // namespace heltec_balancer_ble
}  // namespace esphome

#endif
