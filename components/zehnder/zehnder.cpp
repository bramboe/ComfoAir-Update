#include "zehnder.h"
#include "esphome/core/log.h"
#include "esphome/core/application.h"

namespace esphome {
namespace zehnder {

static const char *const TAG = "zehnder";

// ... (Your existing typedefs, structs like RfPayloadNetworkJoinOpen, RfPayloadNetworkJoinRequest, 
// RfPayloadNetworkJoinAck, RfPayloadFanSettings, RfPayloadFanSetSpeed, RfPayloadFanSetTimer, RfFrame) ...

ZehnderRF::ZehnderRF() 
  : rf_(nullptr), 
    interval_(1000),
    voltage_(0)  // Initialize voltage_ to a default value (e.g., 0)
{ }

void ZehnderRF::setup() {
  ESP_LOGCONFIG(TAG, "Setting up ZehnderRF...");
  if (this->rf_ == nullptr) {
    ESP_LOGE(TAG, "nRF905 RF object is not set");
    return;
  }

  // ... Your existing configuration code ...

  // You might want to update the voltage_ here based on initial fan state or settings.
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
  traits.supports_speed = true;  // Direct assignment

  // Define support for direction control (assuming not supported)
  traits.supports_direction = false;  // Direct assignment

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

  // ... (Your code to transmit speed and timer settings) ...

  // Update the voltage_ based on the new speed setting
  // You'll need to implement the logic to map speed to voltage here
  // For example:
  switch (speed) {
    case FAN_SPEED_AUTO: 
      setVoltage(0); // Or whatever voltage corresponds to AUTO mode
      break;
    case FAN_SPEED_LOW: 
      setVoltage(30); // Assuming 30% corresponds to 3.0 volts
      break;
    // ... add cases for other speeds ...
    default:
      setVoltage(0); // Handle invalid speeds gracefully
      break;
  }
}

void ZehnderRF::queryDevice() {
  // Add code to query the device for status or other information
  // Example: this->startTransmit(...);
}

// ... (Your other existing methods like createDeviceID, discoveryStart, etc.) ...

void ZehnderRF::rfHandleReceived(const uint8_t *const pData, const uint8_t dataLength) {
  // ... (Your existing code to handle received RF data) ...

  // If you receive data that updates the fan speed or settings, 
  // make sure to update the voltage_ accordingly here as well.
}

}  // namespace zehnder
}  // namespace esphome
