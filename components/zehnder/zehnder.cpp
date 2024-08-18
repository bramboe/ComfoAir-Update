#include "zehnder.h"
#include "esphome/core/log.h"
#include "esphome/core/application.h"

namespace esphome {
namespace zehnder {

#define MAX_TRANSMIT_TIME 2000

static const char *const TAG = "zehnder";

ZehnderRF::ZehnderRF() : error_code_(NO_ERROR) {}

void ZehnderRF::setup() {
  ESP_LOGCONFIG(TAG, "ZEHNDER '%s':", this->get_name().c_str());

  memset(&this->config_, 0, sizeof(Config));

  uint32_t hash = fnv1_hash("zehnderrf");
  this->pref_ = global_preferences->make_preference<Config>(hash, true);
  if (this->pref_.load(&this->config_)) {
    ESP_LOGD(TAG, "Config load ok");
  } else {
    ESP_LOGE(TAG, "Failed to load configuration, using defaults.");
    this->error_code_ = E01_COMMUNICATION_ERROR;  // Set error state
  }

  nrf905::Config rfConfig;
  rfConfig = this->rf_->getConfig();
  rfConfig.band = true;
  rfConfig.channel = 118;
  rfConfig.crc_enable = true;
  rfConfig.crc_bits = 16;
  rfConfig.tx_power = 10;
  rfConfig.rx_power = nrf905::PowerNormal;
  rfConfig.rx_address = 0x89816EA9;  // ZEHNDER_NETWORK_LINK_ID;
  rfConfig.rx_address_width = 4;
  rfConfig.rx_payload_width = 16;
  rfConfig.tx_address_width = 4;
  rfConfig.tx_payload_width = 16;
  rfConfig.xtal_frequency = 16000000;  // defaults for now
  rfConfig.clkOutFrequency = nrf905::ClkOut500000;
  rfConfig.clkOutEnable = false;

  this->rf_->updateConfig(&rfConfig);
  this->rf_->writeTxAddress(0x89816EA9);

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

void ZehnderRF::dump_config() {
  ESP_LOGCONFIG(TAG, "Zehnder Fan config:");
  ESP_LOGCONFIG(TAG, "  Polling interval   %u", this->interval_);
  ESP_LOGCONFIG(TAG, "  Fan networkId      0x%08X", this->config_.fan_networkId);
  ESP_LOGCONFIG(TAG, "  Fan my device type 0x%02X", this->config_.fan_my_device_type);
  ESP_LOGCONFIG(TAG, "  Fan my device id   0x%02X", this->config_.fan_my_device_id);
  ESP_LOGCONFIG(TAG, "  Fan main_unit type 0x%02X", this->config_.fan_main_unit_type);
  ESP_LOGCONFIG(TAG, "  Fan main unit id   0x%02X", this->config_.fan_main_unit_id);

  switch (this->error_code_) {
    case NO_ERROR:
      ESP_LOGI(TAG, "No errors detected.");
      break;
    case E01_COMMUNICATION_ERROR:
      ESP_LOGE(TAG, "Communication error.");
      break;
    case E02_TEMPERATURE_SENSOR_FAILURE:
      ESP_LOGE(TAG, "Temperature sensor failure.");
      break;
    case E03_FAN_MALFUNCTION:
      ESP_LOGE(TAG, "Fan malfunction.");
      break;
    case E04_BYPASS_VALVE_ISSUE:
      ESP_LOGE(TAG, "Bypass valve issue.");
      break;
    case E05_FILTER_REPLACEMENT_NEEDED:
      ESP_LOGE(TAG, "Filter replacement needed.");
      break;
    default:
      ESP_LOGE(TAG, "Unknown error code.");
      break;
  }
}

void ZehnderRF::control(const fan::FanCall &call) {
  if (call.get_state().has_value()) {
    this->state = *call.get_state();
    ESP_LOGD(TAG, "Control has state: %u", this->state);
  }
  if (call.get_speed().has_value()) {
    this->speed = *call.get_speed();
    ESP_LOGD(TAG, "Control has speed: %u", this->speed);
  }

  switch (this->state_) {
    case StateIdle:
      this->setSpeed(this->state ? this->speed : 0x00, 0);
      this->lastFanQuery_ = millis();  // Update time
      break;
    default:
      break;
  }

  this->publish_state();
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
          this->rf_->updateConfig(&rfConfig);
          this->rf_->writeTxAddress(this->config_.fan_networkId);
          this->queryDevice();
        }
      }
      break;

    case StateStartDiscovery:
      deviceId = this->createDeviceID();
      this->discoveryStart(deviceId);
      break;

    case StateIdle:
      if (newSetting) {
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
      break;

    default:
      break;
  }
}

void ZehnderRF::rfHandleReceived(const uint8_t *const pData, const uint8_t dataLength) {
  const RfFrame *const pResponse = (RfFrame *) pData;
  RfFrame *const pTxFrame = (RfFrame *) this->_txFrame;  // frame helper

  ESP_LOGD(TAG, "Current state: 0x%02X", this->state_);
  switch (this->state_) {
    case StateDiscoveryWaitForLinkRequest:
      if (dataLength == sizeof(RfPayloadNetworkJoinRequest)) {
        if (pResponse->payload.networkJoinRequest.networkId == this->config_.fan_networkId) {
          if (this->config_.fan_main_unit_type == pResponse->tx_id) {
            pTxFrame->rx_type = 0x02;
            pTxFrame->rx_id = this->config_.fan_my_device_type;
            pTxFrame->tx_type = pResponse->tx_id;
            pTxFrame->tx_id = pResponse->rx_id;
            pTxFrame->ttl = 0x03;
            pTxFrame->command = 0x0C;
            pTxFrame->parameter_count = 0;
            this->rf_->send((uint8_t *) pTxFrame, sizeof(RfFrame));
            this->state_ = StateIdle;
          }
        }
      }
      break;

    case StateDiscoveryWaitForNetworkJoinOpen:
      if (dataLength == sizeof(RfPayloadNetworkJoinOpen)) {
        if (pResponse->payload.networkJoinOpen.networkId == this->config_.fan_networkId) {
          pTxFrame->rx_type = 0x02;
          pTxFrame->rx_id = this->config_.fan_my_device_type;
          pTxFrame->tx_type = pResponse->tx_id;
          pTxFrame->tx_id = pResponse->rx_id;
          pTxFrame->ttl = 0x03;
          pTxFrame->command = 0x07;
          pTxFrame->parameter_count = 3;
          pTxFrame->payload.fanSettings.speed = this->speed_count_;
          pTxFrame->payload.fanSettings.voltage = 0xFF;
          pTxFrame->payload.fanSettings.timer = 0xFF;
          this->rf_->send((uint8_t *) pTxFrame, sizeof(RfFrame));
          this->state_ = StateIdle;
        }
      }
      break;

    case StateIdle:
      break;

    case StateWaitSetSpeedConfirm:
      break;

    default:
      break;
  }
}

void ZehnderRF::rfHandler(void) {
  uint32_t currentMillis = millis();

  if (this->rfState_ == RfStateRxWait) {
    if ((currentMillis - this->msgSendTime_) > MAX_TRANSMIT_TIME) {
      this->rfState_ = RfStateIdle;
      this->retries_--;
      if (this->retries_ >= 0) {
        this->msgSendTime_ = millis();
        this->rfState_ = RfStateTxBusy;
      } else {
        this->rfState_ = RfStateIdle;
      }
    }
  }
}

void ZehnderRF::setSpeed(const uint8_t speed, const uint8_t timer) {
  if (this->rfState_ == RfStateIdle) {
    RfFrame *const pTxFrame = (RfFrame *) this->_txFrame;

    pTxFrame->rx_type = 0x02;
    pTxFrame->rx_id = this->config_.fan_my_device_type;
    pTxFrame->tx_type = this->config_.fan_main_unit_type;
    pTxFrame->tx_id = this->config_.fan_main_unit_id;
    pTxFrame->ttl = 0x03;
    pTxFrame->command = 0x02;
    pTxFrame->parameter_count = 1;
    pTxFrame->payload.setSpeed.speed = speed;

    this->rf_->send((uint8_t *) pTxFrame, sizeof(RfFrame));
    this->rfState_ = RfStateTxBusy;
    this->retries_ = 3;
  } else {
    this->error_code_ = E01_COMMUNICATION_ERROR;
  }
}

}  // namespace zehnder
}  // namespace esphome
