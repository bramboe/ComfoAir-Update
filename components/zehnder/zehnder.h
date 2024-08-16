#ifndef __COMPONENT_ZEHNDER_H__
#define __COMPONENT_ZEHNDER_H__

#include "esphome/core/component.h"
#include "esphome/core/hal.h"
#include "esphome/components/spi/spi.h"
#include "esphome/components/fan/fan_state.h"
#include "esphome/components/nrf905/nRF905.h"

namespace esphome {
namespace zehnder {

// Constants
#define FAN_FRAMESIZE 16        // Each frame consists of 16 bytes
#define FAN_TX_FRAMES 4         // Retransmit every transmitted frame 4 times
#define FAN_TX_RETRIES 10       // Retry transmission 10 times if no reply is received
#define FAN_TTL 250             // 0xFA, default time-to-live for a frame
#define FAN_REPLY_TIMEOUT 1000  // Wait 1000ms for receiving a reply when doing a network scan

// Fan device types
enum FanDeviceType {
  FAN_TYPE_BROADCAST = 0x00,       // Broadcast to all devices
  FAN_TYPE_MAIN_UNIT = 0x01,       // Fans
  FAN_TYPE_REMOTE_CONTROL = 0x03,  // Remote controls
  FAN_TYPE_CO2_SENSOR = 0x18       // CO2 sensors
};

// Fan commands
enum FanCommand {
  FAN_FRAME_SETVOLTAGE = 0x01,  // Set speed (voltage / percentage)
  FAN_FRAME_SETSPEED = 0x02,    // Set speed (preset)
  FAN_FRAME_SETTIMER = 0x03,    // Set speed with timer
  FAN_NETWORK_JOIN_REQUEST = 0x04,
  FAN_FRAME_SETSPEED_REPLY = 0x05,
  FAN_NETWORK_JOIN_OPEN = 0x06,
  FAN_TYPE_FAN_SETTINGS = 0x07,  // Current settings, sent by fan in reply to 0x01, 0x02, 0x10
  FAN_FRAME_0B = 0x0B,
  FAN_NETWORK_JOIN_ACK = 0x0C,
  FAN_TYPE_QUERY_NETWORK = 0x0D,
  FAN_TYPE_QUERY_DEVICE = 0x10,
  FAN_FRAME_SETVOLTAGE_REPLY = 0x1D
};

// Fan speed presets
enum FanSpeedPreset {
  FAN_SPEED_AUTO = 0x00,    // Off:      0% or  0.0 volt
  FAN_SPEED_LOW = 0x01,     // Low:     30% or  3.0 volt
  FAN_SPEED_MEDIUM = 0x02,  // Medium:  50% or  5.0 volt
  FAN_SPEED_HIGH = 0x03,    // High:    90% or  9.0 volt
  FAN_SPEED_MAX = 0x04      // Max:    100% or 10.0 volt
};

// Network ID constants
#define NETWORK_LINK_ID 0xA55A5AA5
#define NETWORK_DEFAULT_ID 0xE7E7E7E7
#define FAN_JOIN_DEFAULT_TIMEOUT 10000

// Result codes
enum Result {
  ResultOk,
  ResultBusy,
  ResultFailure
};

// Error codes
enum ErrorCode {
  NO_ERROR = 0,
  E01_COMMUNICATION_ERROR = 1,
  E03_FAN_MALFUNCTION = 3,
  E05_FILTER_REPLACEMENT_NEEDED = 5
};

class ZehnderRF : public Component, public fan::Fan {
 public:
  ZehnderRF();

  void setup() override;

  // Setup methods
  void set_rf(nrf905::nRF905 *const pRf) { rf_ = pRf; }
  void set_update_interval(const uint32_t interval) { interval_ = interval; }

  void dump_config() override;

  fan::FanTraits get_traits() override;
  int get_speed_count() const { return this->speed_count_; }

  void loop() override;
  void control(const fan::FanCall &call) override;

  float get_setup_priority() const override { return setup_priority::DATA; }

  void setSpeed(const uint8_t speed, const uint8_t timer = 0);

  bool timer;
  int voltage;

  // Error detection
  uint32_t last_successful_communication{0};
  uint32_t filter_runtime{0};
  uint8_t error_code{NO_ERROR}; // Initialize to NO_ERROR

  // Method to get the error code
  uint8_t get_error_code() const { return error_code; }

 protected:
  void queryDevice();
  uint8_t createDeviceID();
  void discoveryStart(const uint8_t deviceId);

  Result startTransmit(const uint8_t *const pData, const int8_t rxRetries = -1,
                       const std::function<void(void)> callback = nullptr);
  void rfComplete();
  void rfHandler();
  void rfHandleReceived(const uint8_t *const pData, const uint8_t dataLength);

  // Method for updating the error status
  void update_error_status();

  // States for the ZehnderRF component
  enum State {
    StateStartup,
    StateStartDiscovery,
    StateDiscoveryWaitForLinkRequest,
    StateDiscoveryWaitForJoinResponse,
    StateDiscoveryJoinComplete,

    StateIdle,
    StateWaitQueryResponse,
    StateWaitSetSpeedResponse,
    StateWaitSetSpeedConfirm,

    StateNrOf  // Keep last
  };
  State state_{StateStartup};
  int speed_count_{};

  nrf905::nRF905 *rf_;
  uint32_t interval_;

  uint8_t _txFrame[FAN_FRAMESIZE];

  ESPPreferenceObject pref_;

  struct Config {
    uint32_t fan_networkId;      // Fan (Zehnder/BUVA) network ID
    uint8_t fan_my_device_type;  // Fan (Zehnder/BUVA) device type
    uint8_t fan_my_device_id;    // Fan (Zehnder/BUVA) device ID
    uint8_t fan_main_unit_type;  // Fan (Zehnder/BUVA) main unit type
    uint8_t fan_main_unit_id;    // Fan (Zehnder/BUVA) main unit ID
  } config_;

  uint32_t lastFanQuery_{0};
  std::function<void(void)> onReceiveTimeout_ = nullptr;

  uint32_t msgSendTime_{0};
  uint32_t airwayFreeWaitTime_{0};
  int8_t retries_{-1};

  uint8_t newSpeed{0};
  uint8_t newTimer{0};
  bool newSetting{false};

  enum RfState {
    RfStateIdle,            // Idle state
    RfStateWaitAirwayFree,  // Wait for airway to be free
    RfStateTxBusy,          // Transmission busy
    RfStateRxWait,          // Waiting for reception
  } rfState_{RfStateIdle};
};

}  // namespace zehnder
}  // namespace esphome

#endif /* __COMPONENT_ZEHNDER_H__ */
