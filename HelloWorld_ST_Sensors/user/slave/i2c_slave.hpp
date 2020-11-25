#ifndef USER_SLAVE_I2C_SLAVE_H_
#define USER_SLAVE_I2C_SLAVE_H_

#include "mbed.h"

#include "sgp30.hpp"
#include "spec_co.hpp"

namespace i2c_slave {

class SlaveCommunication {
public:
  SlaveCommunication(PinName sda, PinName scl, spec::CarbonMonoxide &co,
                     sensor::SGP30 &sgp30)
      : slave_(sda, scl), co_(co), sgp30_(sgp30){};
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

  void send_sgp30_co2();
  void send_sgp30_voc();

  void send_hts221_temperature();
  void send_hts221_humidity();
  void send_lps22hb_temperature();
  void send_lps22hb_pressure();
  void send_time_of_flight();
  void send_accelerometer();
  void send_gyroscope();
  void send_magnetometer();

private:
  I2CSlave slave_;
  Thread thread_;

  spec::CarbonMonoxide &co_;
  sensor::SGP30 &sgp30_;
};

} // namespace i2c_slave

#endif
