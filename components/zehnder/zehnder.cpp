#include "zehnder.h"
#include "esphome/core/log.h"
#include "esphome/core/application.h"

namespace esphome {
namespace zehnder {

static const char *const TAG = "zehnder";

// Definitions for RF payloads and frames
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

ZehnderRF::ZehnderRF() : rf_(nullptr), interval_(1000), error_code_(NO_ERROR), speed_count_(4), lastFanQuery_(0) {}

fan::FanTraits ZehnderRF::get_traits() {
  auto traits = fan::FanTraits();
  traits.supports_speed(true);  // Correct method name
  traits.speed_count_ = this->speed_count_;  // Correct member variable name
  return traits;
}

void ZehnderRF::control(const fan::FanCall &call) {
  if (call.get_state().has_value()) {
    this->state = *call.get_state();  // Correct member variable
    ESP_LOGD(TAG, "Control has state: %u", this->state);
  }
  if (call.get_speed().has_value()) {
    this->speed = *call.get_speed();  // Correct member variable
    ESP_LOGD(TAG, "Control has speed: %u", this->speed);
  }

  if (this->state == StateIdle) {
    this->setSpeed(this->state ? this->speed : 0x00, 0);
    this->lastFanQuery_ = millis();  // Update time
  }

  this->publish_state();
}

void ZehnderRF::setup() {
  ESP_LOGCONFIG(TAG, "Setting up Zehnder RF...");
  
  // Clear config
  memset(&this->config_, 0, sizeof(Config));
  
  uint32_t hash = fnv1_hash("zehnderrf");
  this->pref_ = global_preferences->make_preference<Config>(hash, true);
  if (this->pref_.load(&this->config_)) {
    ESP_LOGD(TAG, "Config load ok");
  }
  
  // Set nRF905 config
  nrf905::Config rfConfig;
  rfConfig = this->rf_->getConfig();
  
  rfConfig.band = true;
  rfConfig.channel = 118;
  rfConfig.crc_enable = true;
  rfConfig.crc_bits = 16;
  rfConfig.tx_power = 10;
  rfConfig.rx_power = nrf905::PowerNormal;
  rfConfig.rx_address = 0x89816EA9;  // ZEHNDER_NETWORK_LINK_ID
  rfConfig.rx_address_width = 4;
  rfConfig.rx_payload_width = 16;
  rfConfig.tx_address_width = 4;
  rfConfig.tx_payload_width = 16;
  rfConfig.xtal_frequency = 16000000;  // defaults for now
  rfConfig.clkOutFrequency = nrf905::ClkOut500000;
  rfConfig.clkOutEnable = false;
  
  this->rf_->updateConfig(&rfConfig);
  this->rf_->writeTxAddress(0x89816EA9);
  
  this->rf_->setOnTxReady([this]() {
    ESP_LOGD(TAG, "Tx Ready");
    if (this->rfState_ == RfStateTxBusy) {
      if (this->retries_ >= 0) {
        this->msgSendTime_ = millis();
        this->rfState_ = RfStateRxWait;
      } else {
        this->rfState_ = RfStateIdle;
      }
    }
  });

  this->rf_->setOnRxComplete([this](const uint8_t *const pData, const uint8_t dataLength) {
    ESP_LOGV(TAG, "Received frame");
    this->rfHandleReceived(pData, dataLength);
  });
}

void ZehnderRF::dump_config() {
  ESP_LOGCONFIG(TAG, "Zehnder Fan config:");
  ESP_LOGCONFIG(TAG, "  Polling interval   %u", this->interval_);
  ESP_LOGCONFIG(TAG, "  Fan networkId      0x%08X", this->config_.fan_networkId);
  ESP_LOGCONFIG(TAG, "  Fan my device type 0x%02X", this->config_.fan_my_device_type);
  ESP_LOGCONFIG(TAG, "  Fan my device id   0x%02X", this->config_.fan_my_device_id);
  ESP_LOGCONFIG(TAG, "  Fan main_unit type 0x%02X", this->config_.fan_main_unit_type);
  ESP_LOGCONFIG(TAG, "  Fan main unit id   0x%02X", this->config_.fan_main_unit_id);
}

void ZehnderRF::loop() {
  if (millis() - this->lastFanQuery_ > this->interval_) {
    this->queryDevice();
    this->lastFanQuery_ = millis();
  }
}

void ZehnderRF::rfHandleReceived(const uint8_t *const pData, const uint8_t dataLength) {
  const RfFrame *const pResponse = (RfFrame *)pData;
  RfFrame *const pTxFrame = (RfFrame *)this->_txFrame;
  nrf905::Config rfConfig;

  ESP_LOGD(TAG, "Current state: 0x%02X", this->state);
  switch (this->state) {
    case StateDiscoveryWaitForJoinResponse:
      if (dataLength == sizeof(RfPayloadNetworkJoinOpen)) {
        if (pResponse->payload.networkJoinOpen.networkId == this->config_.fan_networkId) {
          pTxFrame->rx_type = 0x02;  // Example value
          this->rf_->write(reinterpret_cast<const uint8_t*>(pTxFrame), sizeof(RfFrame));  // Use write method
        }
      }
      break;

    case StateIdle:
      switch (pResponse->command) {
        case FAN_SET_SPEED:
          if (pResponse->payload.setSpeed.speed != this->speed) {
            ESP_LOGD(TAG, "Received speed %u", pResponse->payload.setSpeed.speed);
            this->setSpeed(pResponse->payload.setSpeed.speed, pResponse->payload.setSpeed.timer);
          }
          break;

        case FAN_SET_TIMER:
          if (pResponse->payload.setTimer.timer != this->timer) {
            ESP_LOGD(TAG, "Received timer %u", pResponse->payload.setTimer.timer);
            this->setTimer(pResponse->payload.setTimer.timer);
          }
          break;

        case FAN_SETTINGS:
          this->fanSettingsReceived(pResponse->payload.fanSettings);
          break;

        default:
          ESP_LOGD(TAG, "Unknown command: %u", pResponse->command);
          break;
      }
      break;

    default:
      ESP_LOGD(TAG, "Unhandled state: %u", this->state);
      break;
  }
}

void ZehnderRF::queryDevice() {
  RfFrame frame;
  frame.rx_type = 0x00;
  frame.rx_id = 0x00;
  frame.tx_type = 0x00;
  frame.tx_id = 0x00;
  frame.ttl = 0x00;
  frame.command = FAN_QUERY;  // Ensure this is defined
  frame.parameter_count = 0;

  this->rf_->write(reinterpret_cast<const uint8_t*>(&frame), sizeof(frame));  // Use write method
  ESP_LOGD(TAG, "Sent query to device");
}

void ZehnderRF::setSpeed(uint8_t speed, uint8_t timer) {
  RfFrame frame;
  frame.rx_type = 0x00;
  frame.rx_id = 0x00;
  frame.tx_type = 0x00;
  frame.tx_id = 0x00;
  frame.ttl = 0x00;
  frame.command = FAN_SET_SPEED;  // Ensure this is defined
  frame.parameter_count = sizeof(RfPayloadFanSetSpeed);

  frame.payload.setSpeed.speed = speed;
  frame.payload.setSpeed.timer = timer;  // This might not be used depending on the payload

  this->rf_->write(reinterpret_cast<const uint8_t*>(&frame), sizeof(frame));  // Use write method
  ESP_LOGD(TAG, "Sent set speed %u", speed);
}

void ZehnderRF::setTimer(uint8_t timer) {
  RfFrame frame;
  frame.rx_type = 0x00;
  frame.rx_id = 0x00;
  frame.tx_type = 0x00;
  frame.tx_id = 0x00;
  frame.ttl = 0x00;
  frame.command = FAN_SET_TIMER;  // Ensure this is defined
  frame.parameter_count = sizeof(RfPayloadFanSetTimer);

  frame.payload.setTimer.timer = timer;

  this->rf_->write(reinterpret_cast<const uint8_t*>(&frame), sizeof(frame));  // Use write method
  ESP_LOGD(TAG, "Sent set timer %u", timer);
}

ZehnderRF::ErrorCode ZehnderRF::get_error_code() const {
  return this->error_code_;
}

void ZehnderRF::fanSettingsReceived(const RfPayloadFanSettings &settings) {
  // Implementation of fanSettingsReceived
}

}  // namespace zehnder
}  // namespace esphome
