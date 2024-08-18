#include "zehnder.h"
#include "esphome/core/log.h"
#include "esphome/core/application.h"

namespace esphome {
namespace zehnder {

#define MAX_TRANSMIT_TIME 2000

static const char *const TAG = "zehnder";

// Define error codes
enum ErrorCode {
  NO_ERROR = 0,
  INVALID_SPEED,
  TRANSMISSION_ERROR,
  CONFIG_ERROR,
  UNKNOWN_ERROR
};

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

ZehnderRF::ZehnderRF(void) : error_code_(NO_ERROR) {}

fan::FanTraits ZehnderRF::get_traits() { 
  return fan::FanTraits(false, true, false, this->speed_count_); 
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
      // Set speed
      this->setSpeed(this->state ? this->speed : 0x00, 0);

      this->lastFanQuery_ = millis();  // Update time
      break;

    default:
      break;
  }

  this->publish_state();
}

void ZehnderRF::setup() {
  ESP_LOGCONFIG(TAG, "ZEHNDER '%s':", this->get_name().c_str());

  // Clear config
  memset(&this->config_, 0, sizeof(Config));

  uint32_t hash = fnv1_hash("zehnderrf");
  this->pref_ = global_preferences->make_preference<Config>(hash, true);
  if (!this->pref_.load(&this->config_)) {
    ESP_LOGE(TAG, "Failed to load configuration");
    this->error_code_ = CONFIG_ERROR;
  } else {
    ESP_LOGD(TAG, "Config load ok");
  }

  // Set nRF905 config
  nrf905::Config rfConfig;
  rfConfig = this->rf_->getConfig();

  rfConfig.band = true;
  rfConfig.channel = 118;

  // CRC 16
  rfConfig.crc_enable = true;
  rfConfig.crc_bits = 16;

  // TX power 10
  rfConfig.tx_power = 10;

  // RX power normal
  rfConfig.rx_power = nrf905::PowerNormal;

  rfConfig.rx_address = 0x89816EA9;  // ZEHNDER_NETWORK_LINK_ID;
  rfConfig.rx_address_width = 4;
  rfConfig.rx_payload_width = 16;

  rfConfig.tx_address_width = 4;
  rfConfig.tx_payload_width = 16;

  rfConfig.xtal_frequency = 16000000;  // defaults for now
  rfConfig.clkOutFrequency = nrf905::ClkOut500000;
  rfConfig.clkOutEnable = false;

  // Write config back
  if (!this->rf_->updateConfig(&rfConfig)) {
    ESP_LOGE(TAG, "Failed to update RF configuration");
    this->error_code_ = CONFIG_ERROR;
  }

  if (!this->rf_->writeTxAddress(0x89816EA9)) {
    ESP_LOGE(TAG, "Failed to write TX address");
    this->error_code_ = CONFIG_ERROR;
  }

  this->speed_count_ = 4;

  this->rf_->setOnTxReady([this](void) {
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

void ZehnderRF::dump_config(void) {
  ESP_LOGCONFIG(TAG, "Zehnder Fan config:");
  ESP_LOGCONFIG(TAG, "  Polling interval   %u", this->interval_);
  ESP_LOGCONFIG(TAG, "  Fan networkId      0x%08X", this->config_.fan_networkId);
  ESP_LOGCONFIG(TAG, "  Fan my device type 0x%02X", this->config_.fan_my_device_type);
  ESP_LOGCONFIG(TAG, "  Fan my device id   0x%02X", this->config_.fan_my_device_id);
  ESP_LOGCONFIG(TAG, "  Fan main_unit type 0x%02X", this->config_.fan_main_unit_type);
  ESP_LOGCONFIG(TAG, "  Fan main unit id   0x%02X", this->config_.fan_main_unit_id);
}

void ZehnderRF::loop(void) {
  uint8_t deviceId;
  nrf905::Config rfConfig;

  // Run RF handler
  this->rfHandler();

  switch (this->state_) {
    case StateStartup:
      // Wait until started up
      if (millis() > 15000) {
        // Discovery?
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

          // Start with query
          this->queryDevice();
        }
      }
      break;

    case StateStartDiscovery:
      deviceId = this->createDeviceID();
      this->discoveryStart(deviceId);

      // For now just set TX
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
        // When done, return to idle
        this->state_ = StateIdle;
      }

    default:
      break;
  }
}

void ZehnderRF::rfHandleReceived(const uint8_t *const pData, const uint8_t dataLength) {
  const RfFrame *const pResponse = (RfFrame *) pData;
  RfFrame *const pTxFrame = (RfFrame *) this->_txFrame;  // frame helper
  nrf905::Config rfConfig;

  ESP_LOGD(TAG, "Current state: 0x%02X", this->state_);
  switch (this->state_) {
    case StateDiscoveryWaitForLinkRequest:
      ESP_LOGD(TAG, "DiscoverStateWaitForLinkRequest");
      switch (pResponse->command) {
        case FAN_NETWORK_JOIN_OPEN:  // Received linking request from main unit
          ESP_LOGD(TAG, "Discovery: Found unit type 0x%02X (%s) with ID 0x%02X on network 0x%08X", pResponse->payload.networkJoinOpen.networkId);
          this->discoverySetUnit(pResponse->payload.networkJoinOpen.networkId);
          this->state_ = StateDiscoveryLinkSendAck;
          break;

        default:
          break;
      }
      break;

    case StateDiscoveryLinkSendAck:
      ESP_LOGD(TAG, "DiscoverStateLinkSendAck");
      switch (pResponse->command) {
        case FAN_NETWORK_JOIN_ACK:  // Received ACK from the main unit
          ESP_LOGD(TAG, "Discovery: Received network join ack from main unit");
          rfConfig = this->rf_->getConfig();
          rfConfig.rx_address = this->config_.fan_networkId;
          this->rf_->updateConfig(&rfConfig);
          this->rf_->writeTxAddress(this->config_.fan_networkId);

          // Query device to start polling
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
  this->publish_state();
}

void ZehnderRF::queryDevice(void) {
  // Set default payload
  RfFrame frame;
  frame.rx_type = 0x00;
  frame.rx_id = 0x00;
  frame.tx_type = 0x00;
  frame.tx_id = 0x00;
  frame.ttl = 0x00;
  frame.command = FAN_QUERY;  // Command to query the device
  frame.parameter_count = 0;

  if (!this->rf_->send(reinterpret_cast<const uint8_t*>(&frame), sizeof(frame))) {
    ESP_LOGE(TAG, "Failed to send query to device");
    this->error_code_ = TRANSMISSION_ERROR;
  } else {
    ESP_LOGD(TAG, "Sent query to device");
  }
}

void ZehnderRF::setSpeed(uint8_t speed, uint8_t timer) {
  if (speed > MAX_SPEED) {
    ESP_LOGE(TAG, "Invalid speed %u", speed);
    this->error_code_ = INVALID_SPEED;
    return;
  }

  // Set default payload
  RfFrame frame;
  frame.rx_type = 0x00;
  frame.rx_id = 0x00;
  frame.tx_type = 0x00;
  frame.tx_id = 0x00;
  frame.ttl = 0x00;
  frame.command = FAN_SET_SPEED;
  frame.parameter_count = sizeof(RfPayloadFanSetSpeed);

  frame.payload.setSpeed.speed = speed;
  frame.payload.setSpeed.timer = timer;

  if (!this->rf_->send(reinterpret_cast<const uint8_t*>(&frame), sizeof(frame))) {
    ESP_LOGE(TAG, "Failed to send set speed %u", speed);
    this->error_code_ = TRANSMISSION_ERROR;
  } else {
    ESP_LOGD(TAG, "Sent set speed %u", speed);
  }
}

void ZehnderRF::setTimer(uint8_t timer) {
  // Set default payload
  RfFrame frame;
  frame.rx_type = 0x00;
  frame.rx_id = 0x00;
  frame.tx_type = 0x00;
  frame.tx_id = 0x00;
  frame.ttl = 0x00;
  frame.command = FAN_SET_TIMER;
  frame.parameter_count = sizeof(RfPayloadFanSetTimer);

  frame.payload.setTimer.timer = timer;

  if (!this->rf_->send(reinterpret_cast<const uint8_t*>(&frame), sizeof(frame))) {
    ESP_LOGE(TAG, "Failed to send set timer %u", timer);
    this->error_code_ = TRANSMISSION_ERROR;
  } else {
    ESP_LOGD(TAG, "Sent set timer %u", timer);
  }
}

}  // namespace zehnder
}  // namespace esphome
