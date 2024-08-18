#include "zehnder.h"
#include "esphome/core/log.h"
#include "esphome/components/nrf905/nrf905.h"

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
    RfPayloadNetworkJoinAck networkJoinAck;          // Command 0x0C
  } payload;
} RfFrame;

void ZehnderRF::setup() {
  ESP_LOGCONFIG(TAG, "ZEHNDER '%s':", this->get_name().c_str());

  // Clear config
  memset(&this->config_, 0, sizeof(Config));

  uint32_t hash = fnv1_hash("zehnderrf");
  this->pref_ = global_preferences->make_preference<Config>(hash, true);
  if (!this->pref_.load(&this->config_)) {
    ESP_LOGE(TAG, "Failed to load config");
    this->handle_error(CONFIG_ERROR);
  } else {
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
    ESP_LOGE(TAG, "Failed to update RF config");
    this->handle_error(CONFIG_ERROR);
  }
  if (!this->rf_->writeTxAddress(0x89816EA9)) {
    ESP_LOGE(TAG, "Failed to write TX address");
    this->handle_error(CONFIG_ERROR);
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
            ESP_LOGE(TAG, "Failed to update RF config during startup");
            this->handle_error(CONFIG_ERROR);
          }
          if (!this->rf_->writeTxAddress(this->config_.fan_networkId)) {
            ESP_LOGE(TAG, "Failed to write TX address during startup");
            this->handle_error(CONFIG_ERROR);
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
      break;

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
        case FAN_NETWORK_JOIN_OPEN:
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
        case FAN_NETWORK_JOIN_ACK:
          ESP_LOGD(TAG, "Discovery: Received network join ack from main unit");
          rfConfig = this->rf_->getConfig();
          rfConfig.rx_address = this->config_.fan_networkId;
          if (!this->rf_->updateConfig(&rfConfig)) {
            ESP_LOGE(TAG, "Failed to update RF config during discovery");
            this->handle_error(CONFIG_ERROR);
          }
          if (!this->rf_->writeTxAddress(this->config_.fan_networkId)) {
            ESP_LOGE(TAG, "Failed to write TX address during discovery");
            this->handle_error(CONFIG_ERROR);
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
          ESP_LOGW(TAG, "Unknown command: %d", pResponse->command);
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
    this->handle_error(CONFIG_ERROR);
  }
  ESP_LOGD(TAG, "Sent query to device");
}

void ZehnderRF::setSpeed(uint8_t speed, uint8_t timer) {
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
    ESP_LOGE(TAG, "Failed to send set speed command");
    this->handle_error(CONFIG_ERROR);
  }
  ESP_LOGD(TAG, "Sent set speed %u", speed);
}

void ZehnderRF::setTimer(uint8_t timer) {
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
    ESP_LOGE(TAG, "Failed to send set timer command");
    this->handle_error(CONFIG_ERROR);
  }
  ESP_LOGD(TAG, "Sent set timer %u", timer);
}

void ZehnderRF::handle_error(ErrorCode code) {
  switch (code) {
    case CONFIG_ERROR:
      ESP_LOGE(TAG, "Configuration error");
      break;
    default:
      ESP_LOGE(TAG, "Unknown error code: %d", code);
      break;
  }
}

}  // namespace zehnder
}  // namespace esphome
