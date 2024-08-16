#include "zehnder.h"
#include "esphome/core/log.h"
#include "esphome/core/application.h"

namespace esphome {
namespace zehnder {

#define MAX_TRANSMIT_TIME 2000

static const char *const TAG = "zehnder";

// Define RF payloads
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

ZehnderRF::ZehnderRF() : state_(StateIdle), retries_(10), rfState_(RfStateIdle), last_successful_communication(0), lastFanQuery_(0), error_code_(NO_ERROR) {
  this->set_update_interval(1000);  // Set default update interval
}

fan::FanTraits ZehnderRF::get_traits() {
  return fan::FanTraits(false, true, false, this->speed_count_);
}

void ZehnderRF::control(const fan::FanCall &call) {
  if (call.get_state().has_value()) {
    this->state_ = *call.get_state();
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
      this->lastFanQuery_ = millis();  // Update time
      this->newSetting = false;
      break;

    default:
      break;
  }

  this->last_successful_communication = millis();
  this->publish_state();
}

void ZehnderRF::setup() {
  ESP_LOGCONFIG(TAG, "ZEHNDER '%s':", this->get_name().c_str());

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
}

void ZehnderRF::loop() {
  uint8_t deviceId;
  nrf905::Config rfConfig;

  // Run RF handler
  this->rfHandler();

  // Update error status
  this->update_error_status();

  switch (this->state_) {
    case StateStartup:
      // Wait until started up
      if (millis() > 15000) {
        // Discovery?
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

    case StatePolling:
      // Code for polling state
      if (millis() - this->lastFanQuery_ > 10000) {
        // Example polling condition
        this->queryDevice();
        this->lastFanQuery_ = millis();
      }
      break;

    default:
      ESP_LOGW(TAG, "Unhandled state: %u", this->state_);
      break;
  }
}

void ZehnderRF::update_error_status() {
  // Set error_code_ based on actual device state
  if (/* Communication error condition */) {
    this->error_code_ = E01_COMMUNICATION_ERROR;
  } else if (/* Fan malfunction condition */) {
    this->error_code_ = E03_FAN_MALFUNCTION;
  } else if (/* Filter replacement needed condition */) {
    this->error_code_ = E05_FILTER_REPLACEMENT_NEEDED;
  } else {
    this->error_code_ = NO_ERROR;
  }
}

// Placeholder for the RF handler implementation
void ZehnderRF::rfHandler() {
  // Implement RF handling logic here
}

// Placeholder for the device query implementation
void ZehnderRF::queryDevice() {
  // Implement device query logic here
}

// Placeholder for the discovery start implementation
void ZehnderRF::discoveryStart(uint8_t deviceId) {
  // Implement discovery start logic here
}

// Placeholder for the setSpeed implementation
void ZehnderRF::setSpeed(uint8_t speed, uint8_t timer) {
  // Implement setSpeed logic here
}

}  // namespace zehnder
}  // namespace esphome
