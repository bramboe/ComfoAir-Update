#include "zehnder.h"
#include "esphome/core/log.h"
#include "esphome/core/application.h"

namespace esphome {
namespace zehnder {

#define MAX_TRANSMIT_TIME 2000

static const char *const TAG = "zehnder";

typedef struct __attribute__((packed)) {
  uint32_t networkId;
} RfPayloadNetworkJoinOpen;

typedef struct __attribute__((packed)) {
  uint32_t networkId;
} RfPayloadNetworkJoinRequest;

typedef struct __attribute__((packed)) {
  uint32_t networkId;
} RfPayloadNetworkJoinAck;

typedef struct __attribute__((packed)) {
  uint8_t speed;
  uint8_t voltage;
  uint8_t timer;
} RfPayloadFanSettings;

typedef struct __attribute__((packed)) {
  uint8_t speed;
} RfPayloadFanSetSpeed;

typedef struct __attribute__((packed)) {
  uint8_t speed;
  uint8_t timer;
} RfPayloadFanSetTimer;

typedef struct __attribute__((packed)) {
  uint8_t rx_type;          // 0x00 RX Type
  uint8_t rx_id;            // 0x01 RX ID
  uint8_t tx_type;          // 0x02 TX Type
  uint8_t tx_id;            // 0x03 TX ID
  uint8_t ttl;              // 0x04 Time-To-Live
  uint8_t command;          // 0x05 Frame type
  uint8_t parameter_count;  // 0x06 Number of parameters

  union {
    uint8_t parameters[9];                           // 0x07 - 0x0F Depends on command
    RfPayloadFanSetSpeed setSpeed;                   // Command 0x02
    RfPayloadFanSetTimer setTimer;                   // Command 0x03
    RfPayloadNetworkJoinRequest networkJoinRequest;  // Command 0x04
    RfPayloadNetworkJoinOpen networkJoinOpen;        // Command 0x06
    RfPayloadFanSettings fanSettings;                // Command 0x07
    RfPayloadNetworkJoinAck networkJoinAck;          // Command 0x0C
  } payload;
} RfFrame;

ZehnderRF::ZehnderRF(void) {}

fan::FanTraits ZehnderRF::get_traits() { return fan::FanTraits(false, true, true, this->get_speed_count()); }

void ZehnderRF::setup() {
  ESP_LOGCONFIG(TAG, "Setting up Zehnder Fan...");

  this->pref_ = global_preferences->make_preference<Config>(this->get_object_id_hash());
  if (this->pref_.load(&this->config_)) {
    ESP_LOGI(TAG, "Preferences loaded successfully");
    state_ = StateIdle;
  } else {
    ESP_LOGI(TAG, "No existing preferences found, starting fresh");
    state_ = StateStartup;
  }

  rf_->setReceiveCb([this]() { this->rfHandler(); });
}

void ZehnderRF::control(const fan::FanCall &call) {
  if (call.get_state().has_value()) {
    if (*call.get_state()) {
      if (call.get_speed().has_value()) {
        uint8_t speed = *call.get_speed();
        if (speed < this->speed_count_) {
          this->setSpeed(speed);
        } else {
          ESP_LOGE(TAG, "Invalid speed %d", speed);
          this->error_code_ = E03_FAN_MALFUNCTION;
        }
      } else {
        this->setSpeed(FAN_SPEED_MEDIUM);  // Default to medium speed
      }
    } else {
      this->setSpeed(FAN_SPEED_AUTO);  // Turn off
    }
  }

  if (call.get_oscillating().has_value()) {
    // Handle oscillation (if applicable)
  }

  if (call.get_direction().has_value()) {
    // Handle direction (if applicable)
  }
}

void ZehnderRF::loop() {
  // Periodic tasks, such as querying the device
  if (state_ == StateIdle && (millis() - this->lastFanQuery_) > this->interval_) {
    this->queryDevice();
    this->lastFanQuery_ = millis();
  }
}

void ZehnderRF::dump_config() {
  ESP_LOGCONFIG(TAG, "Zehnder Fan:");
  ESP_LOGCONFIG(TAG, "  Network ID: 0x%08X", this->config_.fan_networkId);
  ESP_LOGCONFIG(TAG, "  Device Type: 0x%02X", this->config_.fan_my_device_type);
  ESP_LOGCONFIG(TAG, "  Device ID: 0x%02X", this->config_.fan_my_device_id);
  ESP_LOGCONFIG(TAG, "  Main Unit Type: 0x%02X", this->config_.fan_main_unit_type);
  ESP_LOGCONFIG(TAG, "  Main Unit ID: 0x%02X", this->config_.fan_main_unit_id);
  ESP_LOGCONFIG(TAG, "  Error Code: %d", this->get_error_code());
}

void ZehnderRF::queryDevice(void) {
  ESP_LOGD(TAG, "Querying device for status...");
  // Implementation of device query
}

void ZehnderRF::setSpeed(const uint8_t speed, const uint8_t timer) {
  ESP_LOGI(TAG, "Setting fan speed to %d with timer %d", speed, timer);

  RfFrame frame = {};
  frame.command = FAN_FRAME_SETSPEED;
  frame.payload.setSpeed.speed = speed;
  frame.ttl = FAN_TTL;

  if (this->startTransmit(reinterpret_cast<const uint8_t *>(&frame), FAN_TX_RETRIES, nullptr) != ResultOk) {
    this->error_code_ = E01_COMMUNICATION_ERROR;
    ESP_LOGE(TAG, "Failed to transmit set speed command.");
  }
}

Result ZehnderRF::startTransmit(const uint8_t *const pData, const int8_t rxRetries,
                                const std::function<void(void)> callback) {
  // Implement the transmit process with retries
  // If fails, set error_code_ to E01_COMMUNICATION_ERROR
  return ResultOk;  // or ResultFailure if there is a problem
}

void ZehnderRF::rfComplete(void) {
  // Called when RF communication completes
  // Handle success or failure here and update error_code_ if needed
}

void ZehnderRF::rfHandler(void) {
  // Handle received data and errors in communication
  // If data is invalid, set error_code_ to E01_COMMUNICATION_ERROR
}

}  // namespace zehnder
}  // namespace esphome
