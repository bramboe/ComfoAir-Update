#include "zehnder.h"
#include "esphome/core/log.h"
#include "esphome/core/application.h"

namespace esphome {
namespace zehnder {

// ... (existing code)

void ZehnderRF::rfHandleReceived(const uint8_t *const pData, const uint8_t dataLength) {
  const RfFrame *const pResponse = (RfFrame *) pData;
  RfFrame *const pTxFrame = (RfFrame *) this->_txFrame;  // frame helper
  nrf905::Config rfConfig;

  // Reset error code at the beginning of each received message
  this->current_error_code_ = 0;

  ESP_LOGD(TAG, "Current state: 0x%02X", this->state_);
  switch (this->state_) {
    // ... (existing cases)

    case StateWaitQueryResponse:
      if ((pResponse->rx_type == this->config_.fan_my_device_type) &&  // If type
          (pResponse->rx_id == this->config_.fan_my_device_id)) {      // and id match, it is for us
        switch (pResponse->command) {
          case FAN_TYPE_FAN_SETTINGS:
            ESP_LOGD(TAG, "Received fan settings; speed: 0x%02X voltage: %i timer: %i",
                     pResponse->payload.fanSettings.speed, pResponse->payload.fanSettings.voltage,
                     pResponse->payload.fanSettings.timer);

            this->rfComplete();

            this->state = pResponse->payload.fanSettings.speed > 0;
            this->speed = pResponse->payload.fanSettings.speed;
            this->timer = pResponse->payload.fanSettings.timer;
            this->voltage = pResponse->payload.fanSettings.voltage;

            // Check for potential error conditions
            if (this->state && this->speed == 0) {
              this->current_error_code_ = 3;  // E03: Fan malfunction
            } else if (this->voltage == 0 && this->speed > 0) {
              this->current_error_code_ = 1;  // E01: Communication error
            }

            this->publish_state();

            this->state_ = StateIdle;
            break;

          // ... (other cases)
        }
      } else {
        ESP_LOGD(TAG, "Received frame from unknown device; type 0x%02X from ID 0x%02X type 0x%02X", pResponse->command,
                 pResponse->tx_id, pResponse->tx_type);
        this->current_error_code_ = 1;  // E01: Communication error
      }
      break;

    // ... (other cases)
  }
}

// ... (rest of the existing code)

}  // namespace zehnder
}  // namespace esphome
