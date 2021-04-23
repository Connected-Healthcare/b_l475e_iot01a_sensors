#ifndef USER_SLAVE_I2C_SLAVE_H_
#define USER_SLAVE_I2C_SLAVE_H_

#include "mbed.h"

#include "user/gps/gps.hpp"
#include "user/heartbeat/heartbeat.hpp"
#include "user/spec/spec_co.hpp"

namespace i2c_slave {

class SlaveCommunication {
public:
  SlaveCommunication(PinName sda, PinName scl, spec::CarbonMonoxide &co,
                     heartbeat::sparkfun_MAX32664 &hb, gps::adafruit_PA6H &gps)
      : slave_(sda, scl), co_(co), hb_(hb), gps_(gps) {
    slave_.frequency(100000);
  };
  void init_thread();

public:
  constexpr static uint32_t ARR_SIZE = 4;
  constexpr static uint32_t I2C_SLAVE_8BIT_ADDRESS = 0xA0;

private:
  void handle_data();
  void transmit_sensor_data(uint8_t address);

  void send_spec_co_gas_concentration();
  void send_spec_co_temperature();
  void send_spec_co_humidity();

  void send_hts221_temperature();
  void send_hts221_humidity();
  void send_lps22hb_temperature();
  void send_lps22hb_pressure();
  void send_time_of_flight();
  void send_accelerometer();
  void send_gyroscope();
  void send_magnetometer();

  void send_heartbeat_data();

  void send_gps_coordinates_data();

private:
  I2CSlave slave_;
  Thread thread_;
  spec::CarbonMonoxide &co_;
  heartbeat::sparkfun_MAX32664 &hb_;
  gps::adafruit_PA6H &gps_;
};

} // namespace i2c_slave

#endif
