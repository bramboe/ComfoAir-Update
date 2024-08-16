#include "zehnder.h"
#include "esphome/core/log.h"
#include "esphome/core/application.h"

namespace esphome {
namespace zehnder {

#define MAX_TRANSMIT_TIME 2000

static const char *const TAG = "zehnder";

/* RF Frame structure */
typedef struct {
  union {
    uint32_t networkId;  // Fan (Zehnder/BUVA) network ID
    struct {
      uint8_t deviceType;
      uint8_t deviceId;
    };
  } header;
  union {
    struct {
      uint8_t unknown;
      uint8_t command;
      uint8_t flags;
      uint8_t speed;
      uint8_t unknown2;
      uint8_t timer;
    } parameters;
    struct {
      uint8_t deviceType;
      uint8_t deviceId;
      uint8_t unknown;
      uint8_t command;
      uint8_t checksum;
    } joinRequest;
  } payload;
} RfFrame;

ZehnderRF::ZehnderRF(void) : errorCode_(0) {}

void ZehnderRF::control(const fan::FanCall &call) {
  if (call.get_state().has_value()) {
    bool new_state = call.get_state().value();
    if (new_state != this->state) {
      this->state = new_state;
      this->newSetting = true;
      this->publish_state(new_state);
      ESP_LOGI(TAG, "Setting state to %s", ONOFF(new_state));
      this->newSpeed = new_state ? FAN_SPEED_AUTO : FAN_SPEED_LOW;
      // If we change state, we should also query the device for the new settings to reflect the state
      if (this->state_ == StateIdle) {
        this->queryDevice();
      }
    }
  }

  if (call.get_speed().has_value()) {
    float new_speed = call.get_speed().value();

    uint8_t speed = 0;
    if (new_speed <= 0.3f) {
      speed = FAN_SPEED_LOW;
    } else if (new_speed <= 0.5f) {
      speed = FAN_SPEED_MEDIUM;
    } else if (new_speed <= 0.9f) {
      speed = FAN_SPEED_HIGH;
    } else {
      speed = FAN_SPEED_MAX;
    }

    if (speed != this->speed) {
      this->speed = speed;
      this->newSpeed = speed;
      this->newSetting = true;
      ESP_LOGI(TAG, "Setting speed to %d", this->speed);
      // If we change speed, we should also query the device for the new settings to reflect the speed
      if (this->state_ == StateIdle) {
        this->queryDevice();
      }
    }
  }
}

void ZehnderRF::setup() {
  ESP_LOGCONFIG(TAG, "Setting up Zehnder ComfoConnect bridge...");

  this->speed_count_ = 4;
  this->pref_.begin("zehnder");
  this->config_ = this->pref_.read<Config>();

  if (this->config_.fan_networkId == 0) {
    this->config_.fan_networkId = NETWORK_DEFAULT_ID;
  }

  ESP_LOGI(TAG, "Network ID: 0x%08X", this->config_.fan_networkId);

  this->rf_->set_frequency(868300000);  // 868.3 MHz
  this->rf_->set_tx_power(10);           // 10 dBm, default
  this->rf_->set_crc_mode(true);
  this->rf_->set_address_width(4);
  this->rf_->set_irq_config(false, true, true);

  this->rf_->set_receive_mode();
  this->rf_->set_callback(std::bind(&ZehnderRF::rfHandler, this));

  this->set_interval("query_interval", this->interval_, 10000, 60000,
                     [this](uint32_t interval) { this->set_update_interval(interval); });

  this->lastFanQuery_ = 0;
  this->state_ = StateStartDiscovery;
}

void ZehnderRF::dump_config(void) {
  ESP_LOGCONFIG(TAG, "Zehnder:");
  LOG_FAN("  ", "Fan", this);
  ESP_LOGCONFIG(TAG, "  Query interval: %u seconds", this->interval_ / 1000);
  ESP_LOGCONFIG(TAG, "  Main unit ID: %u", this->config_.fan_main_unit_id);
  ESP_LOGCONFIG(TAG, "  Main unit type: %u", this->config_.fan_main_unit_type);
}

fan::FanTraits ZehnderRF::get_traits() {
  auto traits = fan::FanTraits();
  traits.set_supported_speeds({
      fan::FanSpeed::LOW,
      fan::FanSpeed::MEDIUM,
      fan::FanSpeed::HIGH,
  });
  traits.set_supports_oscillation(false);
  traits.set_supports_direction(false);
  return traits;
}

void ZehnderRF::loop(void) {
  const uint32_t now = millis();

  switch (this->state_) {
    case StateStartup: {
      delay(1000);
      this->state_ = StateStartDiscovery;
      break;
    }
    case StateStartDiscovery: {
      uint8_t deviceId = createDeviceID();
      discoveryStart(deviceId);
      break;
    }
    case StateDiscoveryWaitForLinkRequest: {
      if (now - this->msgSendTime_ > FAN_JOIN_DEFAULT_TIMEOUT) {
        ESP_LOGW(TAG, "Timeout waiting for link request from fan");
        this->state_ = StateStartDiscovery;
      }
      break;
    }
    case StateDiscoveryWaitForJoinResponse: {
      if (now - this->msgSendTime_ > FAN_REPLY_TIMEOUT) {
        ESP_LOGW(TAG, "Timeout waiting for join open response from fan");
        this->state_ = StateStartDiscovery;
      }
      break;
    }
    case StateDiscoveryJoinComplete: {
      this->state_ = StateIdle;
      this->queryDevice();
      break;
    }

    case StateIdle: {
      if (newSetting == true) {
        this->setSpeed(newSpeed, newTimer);
      } else {
        if ((now - this->lastFanQuery_) > this->interval_) {
          this->queryDevice();
        }
      }

      // Reset error code after some time in idle state
      if (now - this->lastErrorTime_ > 5000) { // Adjust timeout as needed
        this->errorCode_ = 0;
      }
      break;
    }
    case StateWaitQueryResponse: {
      if (now - this->msgSendTime_ > FAN_REPLY_TIMEOUT) {
        ESP_LOGW(TAG, "Timeout waiting for response to device query from fan");
        this->state_ = StateIdle;
      }
      break;
    }

    case StateWaitSetSpeedResponse: {
      if (now - this->msgSendTime_ > FAN_REPLY_TIMEOUT) {
        ESP_LOGW(TAG, "Timeout waiting for response to speed change request from fan");
        this->state_ = StateIdle;
      }
      break;
    }

    case StateWaitSetSpeedConfirm: {
      if (now - this->msgSendTime_ > FAN_REPLY_TIMEOUT) {
        ESP_LOGW(TAG, "Timeout waiting for confirmation to speed change request from fan");
        this->state_ = StateIdle;
      }
      break;
    }

    case StateNrOf:
      break;
  }

  // If there's a pending timeout callback and it's time, call it
  if ((this->onReceiveTimeout_ != NULL) && (now - this->airwayFreeWaitTime_ > MAX_TRANSMIT_TIME)) {
    this->onReceiveTimeout_();
    this->onReceiveTimeout_ = NULL;
  }
}

void Zeh
