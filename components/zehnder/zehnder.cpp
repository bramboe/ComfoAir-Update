// zehnder.cpp
#include "zehnder.h"
#include "esphome/core/log.h"
#include "esphome/core/application.h"

namespace esphome {
namespace zehnder {

#define MAX_TRANSMIT_TIME 2000

static const char *const TAG = "zehnder";

// Define commands and other constants here
// ...

void ZehnderRF::setup() {
  ESP_LOGCONFIG(TAG, "ZEHNDER '%s':", this->get_name().c_str());

  // Clear config
  memset(&this->config_, 0, sizeof(Config));

  uint32_t hash = fnv1_hash("zehnderrf");
  this->pref_ = global_preferences->make_preference<Config>(hash, true);
  if (this->pref_.load(&this->config_)) {
    ESP_LOGD(TAG, "Config load ok");
  } else {
    this->handle_error(CONFIG_ERROR);
    return;
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

  rfConfig.rx_address = 0x89816EA9;
  rfConfig.rx_address_width = 4;
  rfConfig.rx_payload_width = 16;

  rfConfig.tx_address_width = 4;
  rfConfig.tx_payload_width = 16;

  rfConfig.xtal_frequency = 16000000;
  rfConfig.clkOutFrequency = nrf905::ClkOut500000;
  rfConfig.clkOutEnable = false;

  if (!this->rf_->updateConfig(&rfConfig)) {
    this->handle_error(CONFIG_ERROR);
    return;
  }

  if (!this->rf_->writeTxAddress(0x89816EA9)) {
    this->handle_error(CONFIG_ERROR);
    return;
  }

  this->speed_count_ = 4;

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

void ZehnderRF::handle_error(ErrorCode code) {
  this->error_code_ = code;
  ESP_LOGE(TAG, "Error occurred: %d", code);
}

void ZehnderRF::loop() {
  uint8_t deviceId;
  nrf905::Config rfConfig;

  this->rfHandler();

  switch (this->state_) {
    case StateStartup:
      if (millis() > 15000) {
        if ((this->config_.fan_networkId == 0x00000000) || (this->config_.fan_my_device_type == 0) ||
            (this->config_.fan_my_device_id == 0) || (this->config_.fan_main_unit_type == 0) ||
            (this->config_.fan_main_unit_id == 0)) {
          ESP_LOGD(TAG, "Invalid config, start pairing");

          this->state_ = StateStartDiscovery;
        } else {
          ESP_LOGD(TAG, "Config data valid, start polling");

          rfConfig = this->rf_->getConfig();
          rfConfig.rx_address = this->config_.fan_networkId;
          if (!this->rf_->updateConfig(&rfConfig)) {
            this->handle_error(CONFIG_ERROR);
            return;
          }
          if (!this->rf_->writeTxAddress(this->config_.fan_networkId)) {
            this->handle_error(CONFIG_ERROR);
            return;
          }

          this->queryDevice();
        }
      }
      break;

    case StateStartDiscovery:
      deviceId = this->createDeviceID();
      this->discoveryStart(deviceId);
      break;

    case StateIdle:
      if (newSetting == true) {
        this->setSpeed(newSpeed, newTimer);
      } else {
        if ((millis() - this->lastFanQuery_) > this->interval_) {
          this->queryDevice();
        }
      }
      break;

    case StateWaitSetSpeedConfirm:
      if (this->rfState_ == RfStateIdle) {
        this->state_ = StateIdle;
      }

    default:
      break;
  }
}

void ZehnderRF::rfHandleReceived(const uint8_t *const pData, const uint8_t dataLength) {
  const RfFrame *const pResponse = (RfFrame *) pData;
  nrf905::Config rfConfig;

  ESP_LOGD(TAG, "Current state: 0x%02X", this->state_);
  switch (this->state_) {
    case StateDiscoveryWaitForLinkRequest:
      switch (pResponse->command) {
        case FAN_NETWORK_JOIN_OPEN:
          ESP_LOGD(TAG, "Discovery: Found unit type 0x%02X with ID 0x%02X on network 0x%08X", pResponse->payload.networkJoinOpen.networkId);
          this->discoverySetUnit(pResponse->payload.networkJoinOpen.networkId);
          this->state_ = StateDiscoveryLinkSendAck;
          break;

        default:
          break;
      }
      break;

    case StateDiscoveryLinkSendAck:
      switch (pResponse->command) {
        case FAN_NETWORK_JOIN_ACK:
          ESP_LOGD(TAG, "Discovery: Received network join ack from main unit");
          rfConfig = this->rf_->getConfig();
          rfConfig.rx_address = this->config_.fan_networkId;
          if (!this->rf_->updateConfig(&rfConfig)) {
            this->handle_error(CONFIG_ERROR);
            return;
          }
          if (!this->rf_->writeTxAddress(this->config_.fan_networkId)) {
            this->handle_error(CONFIG_ERROR);
            return;
          }

          this->queryDevice();
          this->state_ = StateIdle;
          break;

        default:
          break;
      }
      break;

    case StateIdle:
      switch (pResponse->command) {
        case FAN_SET_SPEED:
          if (pResponse->payload.setSpeed.speed != this->speed) {
            ESP_LOGD(TAG, "Received speed %u", pResponse->payload.setSpeed.speed);
            this->setSpeed(pResponse->payload.setSpeed.speed, 0);
          }
          break;

        case FAN_SET_TIMER:
          if (pResponse->payload.setTimer.timer != this->timer) {
            ESP_LOGD(TAG, "Received timer %u", pResponse->payload.setTimer.timer);
            this->setTimer(pResponse->payload.setTimer.timer);
          }
          break;

        case FAN_SETTINGS:
          ESP_LOGD(TAG, "Received fan settings");
          this->fanSettingsReceived(pResponse->payload.fanSettings);
          break;

        default:
          break;
      }
      break;

    default:
      break;
  }
}

void ZehnderRF::fanSettingsReceived(const RfPayloadFanSettings &settings) {
  this->speed = settings.speed;
  this->voltage = settings.voltage;
  this->timer = settings.timer;
}

void ZehnderRF::control(const fan::FanCall &call) {
  if (call.get_speed().has_value()) {
    this->setSpeed(call.get_speed().value(), 0);
  }
  if (call.get_timer().has_value()) {
    this->setTimer(call.get_timer().value());
  }
}

void ZehnderRF::setSpeed(uint8_t speed, uint8_t timer) {
  this->state_ = StateWaitSetSpeedConfirm;
  this->msgSendTime_ = millis();

  // Send command to set speed
  // ...

  this->lastFanQuery_ = millis();
}

void ZehnderRF::setTimer(uint8_t timer) {
  this->state_ = StateWaitSetSpeedConfirm;
  this->msgSendTime_ = millis();

  // Send command to set timer
  // ...

  this->lastFanQuery_ = millis();
}

void ZehnderRF::queryDevice(void) {
  this->state_ = StateIdle;
}

}  // namespace zehnder
}  // namespace esphome
