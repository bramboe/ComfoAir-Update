#include "zehnder.h"
#include "esphome/core/log.h"
#include "esphome/core/application.h"

namespace esphome {
namespace zehnder {

static const char *const TAG = "zehnder";

typedef struct {
  uint8_t rx_type;
  union {
    struct {
      uint32_t networkId;
    } networkJoinRequest;
    struct {
      uint32_t networkId;
    } networkJoinOpen;
    // Add other payload structures if needed
  } payload;
} RfFrame;

ZehnderRF::ZehnderRF() : rf_(nullptr), interval_(1000), error_code_(NO_ERROR) {}

void ZehnderRF::setup() {
  ESP_LOGCONFIG(TAG, "Setting up Zehnder RF...");
}

void ZehnderRF::dump_config() {
  ESP_LOGCONFIG(TAG, "Zehnder RF:");
  ESP_LOGCONFIG(TAG, "  Update Interval: %u ms", this->interval_);
}

fan::FanTraits ZehnderRF::get_traits() {
  auto traits = fan::FanTraits();
  traits.set_supports_speed(true);
  traits.set_speed_count(this->speed_count_);
  return traits;
}

void ZehnderRF::loop() {
  if (millis() - this->lastFanQuery_ > this->interval_) {
    this->queryDevice();
    this->lastFanQuery_ = millis();
  }
}

void ZehnderRF::control(const fan::FanCall &call) {
  this->setSpeed(call.get_speed(), call.get_timer());
}

void ZehnderRF::setSpeed(const uint8_t speed, const uint8_t timer) {
  RfFrame *const pTxFrame = (RfFrame *) this->_txFrame;
  pTxFrame->rx_type = FAN_FRAME_SETSPEED;
  pTxFrame->payload.networkJoinRequest.networkId = this->config_.fan_networkId;
  // Set other fields as needed

  this->rf_->transmit((uint8_t *) pTxFrame, sizeof(RfFrame));
}

void ZehnderRF::queryDevice() {
  // Implementation to query the device
}

void ZehnderRF::rfHandleReceived(const uint8_t *const pData, const uint8_t dataLength) {
  const RfFrame *const pResponse = (RfFrame *) pData;
  RfFrame *const pTxFrame = (RfFrame *) this->_txFrame;  // frame helper

  switch (this->state_) {
    case StateDiscoveryWaitForNetworkJoinOpen:
      if (dataLength == sizeof(RfPayloadNetworkJoinOpen)) {
        if (pResponse->payload.networkJoinOpen.networkId == this->config_.fan_networkId) {
          pTxFrame->rx_type = 0x02;  // Example value
          this->rf_->transmit((uint8_t *) pTxFrame, sizeof(RfFrame));
        }
      }
      break;

    // Handle other states and responses
    default:
      break;
  }
}

ZehnderRF::ErrorCode ZehnderRF::get_error_code() const {
  return this->error_code_;
}

}  // namespace zehnder
}  // namespace esphome

