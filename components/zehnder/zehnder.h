#ifndef __COMPONENT_ZEHNDER_H__
#define __COMPONENT_ZEHNDER_H__

#include "esphome/core/component.h"
#include "esphome/core/hal.h"
#include "esphome/components/spi/spi.h"
#include "esphome/components/fan/fan_state.h"
#include "esphome/components/nrf905/nRF905.h"

namespace esphome {
namespace zehnder {

// ... (existing definitions)

class ZehnderRF : public Component, public fan::Fan {
 public:
  ZehnderRF();

  void setup() override;

  // Setup things
  void set_rf(nrf905::nRF905 *const pRf) { rf_ = pRf; }

  void set_update_interval(const uint32_t interval) { interval_ = interval; }

  void dump_config() override;

  fan::FanTraits get_traits() override;
  int get_speed_count() { return this->speed_count_; }

  void loop() override;

  void control(const fan::FanCall &call) override;

  float get_setup_priority() const override { return setup_priority::DATA; }

  void setSpeed(const uint8_t speed, const uint8_t timer = 0);

  bool timer;
  int voltage;

  // New method to get the current error code
  uint8_t get_error_code() const { return current_error_code_; }

 protected:
  void queryDevice(void);

  uint8_t createDeviceID(void);
  void discoveryStart(const uint8_t deviceId);

  Result startTransmit(const uint8_t *const pData, const int8_t rxRetries = -1,
                       const std::function<void(void)> callback = NULL);
  void rfComplete(void);
  void rfHandler(void);
  void rfHandleReceived(const uint8_t *const pData, const uint8_t dataLength);

  // ... (existing enum definitions and member variables)

  // New member variable to store the current error code
  uint8_t current_error_code_{0};
};

}  // namespace zehnder
}  // namespace esphome

#endif /* __COMPONENT_ZEHNDER_H__ */
