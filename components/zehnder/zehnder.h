#ifndef __COMPONENT_ZEHNDER_H__
#define __COMPONENT_ZEHNDER_H__

#include "esphome/core/component.h"
#include "esphome/core/hal.h"
#include "esphome/components/spi/spi.h"
#include "esphome/components/fan/fan_state.h"
#include "esphome/components/nrf905/nRF905.h"

namespace esphome {
namespace zehnder {

#define FAN_FRAMESIZE 16
#define FAN_TX_FRAMES 4
#define FAN_TX_RETRIES 10
#define FAN_TTL 250
#define FAN_REPLY_TIMEOUT 1000

enum {
  FAN_TYPE_BROADCAST = 0x00,
  FAN_TYPE_MAIN_UNIT = 0x01,
  FAN_TYPE_REMOTE_CONTROL = 0x03,
  FAN_TYPE_CO2_SENSOR = 0x18
};

enum {
  FAN_FRAME_SETVOLTAGE = 0x01,
  FAN_FRAME_SETSPEED = 0x02,
  FAN_FRAME_SETTIMER = 0x03,
  FAN_NETWORK_JOIN_REQUEST = 0x04,
  FAN_FRAME_SETSPEED_REPLY = 0x05,
  FAN_NETWORK_JOIN_OPEN = 0x06,
  FAN_TYPE_FAN_SETTINGS = 0x07,
  FAN_FRAME_0B = 0x0B,
  FAN_NETWORK_JOIN_ACK = 0x0C,
  FAN_TYPE_QUERY_NETWORK = 0x0D,
  FAN_TYPE_QUERY_DEVICE = 0x10,
  FAN_FRAME_SETVOLTAGE_REPLY = 0x1D
};

enum {
  FAN_SPEED_AUTO = 0x00,
  FAN_SPEED_LOW = 0x01,
  FAN_SPEED_MEDIUM = 0x02,
  FAN_SPEED_HIGH = 0x03,
  FAN_SPEED_MAX = 0x04
};

#define NETWORK_LINK_ID 0xA55A5AA5
#define NETWORK_DEFAULT_ID 0xE7E7E7E7
#define FAN_JOIN_DEFAULT_TIMEOUT 10000

typedef enum { ResultOk, ResultBusy, ResultFailure } Result;

class ZehnderRF : public Component, public fan::Fan {
 public:
  ZehnderRF();

  void setup() override;
  void loop() override;

  void set_rf(nrf905::nRF905 *const pRf) { rf_ = pRf; }
  void set_update_interval(const uint32_t interval) { interval_ = interval; }

  void dump_config() override;
  fan::FanTraits get_traits() override;

  int get_speed_count() { return this->speed_count_; }

  void control(const fan::FanCall &call) override;
  float get_setup_priority() const override { return setup_priority::DATA; }

  void setSpeed(const uint8_t speed, const uint8_t timer = 0);

  bool timer;
  int getVoltage() const { return this->voltage_; }
  void setVoltage(int newVoltage) { this->voltage_ = newVoltage; }

  enum ErrorCode {
    NO_ERROR = 0,
    E01_COMMUNICATION_ERROR = 1,
    E02_TEMPERATURE_SENSOR_FAILURE = 2,
    E03_FAN_MALFUNCTION = 3,
    E04_BYPASS_VALVE_ISSUE = 4,
    E05_FILTER_REPLACEMENT_NEEDED = 5
  };

  ErrorCode get_error_code() const { return error_code_; }

 protected:
  void queryDevice(void);

  uint8_t createDeviceID(void);
  void discoveryStart(const uint8_t deviceId);

  Result startTransmit(const uint8_t *const pData, const int8_t rxRetries = -1,
                       const std::function<void(void)> callback = NULL);
  void rfComplete(void);
  void rfHandler(void);
  void rfHandleReceived(const uint8_t *const pData, const uint8_t dataLength);

  typedef enum {
    StateStartup,
    StateStartDiscovery,
    StateDiscoveryWaitForLinkRequest,
    StateDiscoveryWaitForJoinResponse,
    StateDiscoveryJoinComplete,
    StateIdle,
    StateWaitQueryResponse,
    StateWaitSetSpeedResponse,
    StateWaitSetSpeedConfirm,
    StateNrOf
  } State;
  State state_{StateStartup};
  int speed_count_{};

  nrf905::nRF905 *rf_;
  uint32_t interval_;

  uint8_t _txFrame[FAN_FRAMESIZE];

  ESPPreferenceObject pref_;

  typedef struct {
    uint32_t fan_networkId;
    uint8_t fan_my_device_type;
    uint8_t fan_my_device_id;
    uint8_t fan_main_unit_type;
    uint8_t fan_main_unit_id;
  } Config;
  Config config_;

  uint32_t lastFanQuery_{0};
  std::function<void(void)> onReceiveTimeout_ = NULL;

  uint32_t msgSendTime_{0};
  uint32_t airwayFreeWaitTime_{0};
  int8_t retries_{-1};

  uint8_t newSpeed{0};
  uint8_t newTimer{0};
  bool newSetting{false};

  typedef enum {
    RfStateIdle,
    RfStateWaitAirwayFree,
    RfStateTxBusy,
    RfStateRxWait,
  } RfState;
  RfState rfState_{RfStateIdle};

  ErrorCode error_code_{NO_ERROR};

 private:
  int voltage_;
};

}  // namespace zehnder
}  // namespace esphome

#endif /* __COMPONENT_ZEHNDER_H__ */
