#include "zehnder.h"
#include "esphome/core/log.h"
#include "esphome/core/application.h"

namespace esphome {
namespace zehnder {

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
  };
} RfFrame;

ZehnderRF::ZehnderRF() : rf_(nullptr), interval_(1000) {}

void ZehnderRF::setup() {
  ESP_LOGCONFIG(TAG, "Setting up ZehnderRF...");
  if (this->rf_ == nullptr) {
    ESP_LOGE(TAG, "nRF905 RF object is not set");
    return;
  }

  // Configuration code here...
  this->queryDevice();
}

void ZehnderRF::dump_config() {
  ESP_LOGCONFIG(TAG, "ZehnderRF:");
  ESP_LOGCONFIG(TAG, "  Device ID: %d", this->config_.fan_my_device_id);
}

fan::FanTraits ZehnderRF::get_traits() {
  fan::FanTraits traits;

  // Define the number of supported speeds
  traits.set_supported_speed_count(5);  // Number of speeds: AUTO, LOW, MEDIUM, HIGH, MAX

  // Define support for speed control
  traits.set_supports_speed(true);
  
  // Define support for direction control (assuming not supported)
  traits.set_supports_direction(false);

  return traits;
}

void ZehnderRF::control(const fan::FanCall &call) {
  if (call.get_state().has_value()) {
    ESP_LOGD(TAG, "Fan state change detected");
    if (call.get_state().value()) {
      // Handle turning the fan on
      if (call.get_speed().has_value()) {
        this->setSpeed(call.get_speed().value());
      }
    } else {
      // Handle turning the fan off
      this->setSpeed(FAN_SPEED_AUTO);
    }
  }
}

void ZehnderRF::loop() {
  if (this->rf_ == nullptr) return;
  
  if (millis() - this->lastFanQuery_ > this->interval_) {
    this->queryDevice();
    this->lastFanQuery_ = millis();
  }
}

void ZehnderRF::setSpeed(const uint8_t speed, const uint8_t timer) {
  ESP_LOGD(TAG, "Setting fan speed: %d, timer: %d", speed, timer);

  this->newSpeed = speed;
  this->newTimer = timer;
  this->newSetting = true;

  // Add code to transmit speed and timer settings to the device
  // Example: this->startTransmit(...);
}

void ZehnderRF::queryDevice() {
  // Add code to query the device for status or other information
  // Example: this->startTransmit(...);
}

}  // namespace zehnder
}  // namespace esphome
