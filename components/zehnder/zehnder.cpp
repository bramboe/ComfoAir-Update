#include "zehnder.h"
#include "esphome/core/log.h"

namespace esphome {
namespace zehnder {

#define MAX_TRANSMIT_TIME 2000

static const char *const TAG = "zehnder";

// Define error codes
enum ErrorCode {
  NO_ERROR = 0,
  E01_COMMUNICATION_ERROR = 1,
  E03_FAN_MALFUNCTION = 3,
  E05_FILTER_REPLACEMENT_NEEDED = 5,
};

// Define states
enum State {
  StateIdle,
  StateActive,
  StatePolling,
  StateTxBusy,
  StateStartup,
  StateStartDiscovery,
};

ZehnderRF::ZehnderRF() 
  : state_(StateStartup), // Initialized to StateStartup for initial setup
    retries_(10), 
    rfState_(RfStateIdle), 
    last_successful_communication(0), 
    lastFanQuery_(0), 
    error_code(NO_ERROR) {
  this->set_update_interval(1000); // Set update interval to 1 second
}

fan::FanTraits ZehnderRF::get_traits() {
  return fan::FanTraits(false, true, false, this->speed_count_);
}

void ZehnderRF::control(const fan::FanCall &call) {
  if (call.get_state().has_value()) {
    this->state_ = *call.get_state() ? StateActive : StateIdle;
    ESP_LOGD(TAG, "Control has state: %u", this->state_);
  }
  if (call.get_speed().has_value()) {
    this->newSpeed = *call.get_speed();
    ESP_LOGD(TAG, "Control has speed: %u", this->newSpeed);
    this->newSetting = true;
  }

  switch (this->state_) {
    case StateIdle:
      this->setSpeed(this->newSetting ? this->newSpeed : 0x00, 0);
      this->lastFanQuery_ = millis();
      this->newSetting = false;
      break;
    case StatePolling:
      // Implement StatePolling behavior
      break;
    case StateTxBusy:
      // Implement StateTxBusy behavior
      break;
    default:
      break;
  }

  this->last_successful_communication = millis();
  this->publish_state();
}

void ZehnderRF::setup() {
  ESP_LOGCONFIG(TAG, "ZEHNDER '%s':", this->get_name().c_str());

  memset(&this->config_, 0, sizeof(Config));

  uint32_t hash = fnv1_hash("zehnderrf");
  this->pref_ = global_preferences->make_preference<Config>(hash, true);
  if (this->pref_.load(&this->config_)) {
    ESP_LOGD(TAG, "Config load ok");
  } else {
    ESP_LOGW(TAG, "Failed to load config");
  }

  nrf905::Config rfConfig;
  rfConfig = this->rf_->getConfig();

  rfConfig.band = true;
  rfConfig.channel = 118;
  rfConfig.crc_enable = true;
  rfConfig.crc_bits = 16;
  rfConfig.tx_power = 10;
  rfConfig.rx_power = nrf905::PowerNormal;
  rfConfig.rx_address = 0x89816EA9;
  rfConfig.rx_address_width = 4;
  rfConfig.rx_payload_width = 16;
  rfConfig.tx_address_width = 4;
  rfConfig.tx_payload_width = 16;
  rfConfig.xtal_frequency = 16000000;
  rfConfig.clkOutFrequency = nrf905::ClkOut500000;
  rfConfig.clkOutEnable = false;

  this->rf_->updateConfig(&rfConfig);
  this->rf_->writeTxAddress(0x89816EA9);

  this->speed_count_ = 4;

  this->rf_->setOnTxReady([this]() {
    ESP_LOGD(TAG, "Tx Ready");
    if (this->rfState_ == RfStateTxBusy) {
      if (this->retries_ > 0) {
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
  uint8_t deviceId;
  nrf905::Config rfConfig;

  this->rfHandler();
  this->update_error_status();

  switch (this->state_) {
    case StateStartup:
      if (millis() > 15000) {
        if ((this->config_.fan_networkId == 0x00000000) || 
            (this->config_.fan_my_device_type == 0) ||
            (this->config_.fan_my_device_id == 0) || 
            (this->config_.fan_main_unit_type == 0) ||
            (this->config_.fan_main_unit_id == 0)) {
          ESP_LOGD(TAG, "Invalid config, start pairing");
          this->state_ = StateStartDiscovery;
        } else {
          ESP_LOGD(TAG, "Config data valid, start polling");
          rfConfig = this->rf_->getConfig();
          rfConfig.rx_address = this->config_.fan_networkId;
          rfConfig.rx_payload_width = 16;
          rfConfig.rx_address_width = 4;
          this->rf_->updateConfig(&rfConfig);
          this->state_ = StatePolling;
        }
      }
      break;

    case StateStartDiscovery:
      this->discoveryStart(deviceId);
      break;

    case StatePolling:
      if (millis() - this->lastFanQuery_ > 30000) {
        if (this->rfState_ == RfStateIdle) {
          this->queryDevice();
        }
      }
      break;

    case StateTxBusy:
      if (millis() - this->msgSendTime_ > MAX_TRANSMIT_TIME) {
        this->rfState_ = RfStateIdle;
        this->retries_--;
      }
      break;

    default:
      break;
  }
}

void ZehnderRF::update_error_status() {
  // Example conditions, update with actual logic
  if (/* Communication error condition */) {
    this->error_code = E01_COMMUNICATION_ERROR;
  } else if (/* Fan malfunction condition */) {
    this->error_code = E03_FAN_MALFUNCTION;
  } else if (/* Filter replacement needed condition */) {
    this->error_code = E05_FILTER_REPLACEMENT_NEEDED;
  } else {
    this->error_code = NO_ERROR;
  }
}

void ZehnderRF::rfHandler() {
  // Handle RF events here
}

void ZehnderRF::queryDevice() {
  // Implement device query logic here
}

void ZehnderRF::discoveryStart(uint8_t deviceId) {
  // Implement discovery start logic here
}

void ZehnderRF::setSpeed(uint8_t speed, uint8_t timer) {
  // Implement setSpeed logic here
}

}  // namespace zehnder
}  // namespace esphome
