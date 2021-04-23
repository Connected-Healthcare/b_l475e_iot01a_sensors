#ifndef USER_SGP30_H_
#define USER_SGP30_H_

#include "mbed.h"

namespace sensor {

class SGP30 {
public:
  SGP30(PinName sda, PinName scl) : i2c_(sda, scl) {
    memset(id, 0, 3 * sizeof(uint16_t));
  }

  void start(void);

  void get_id(void);

  uint16_t get_co2(void) { return co2_; }
  uint16_t get_voc(void) { return voc_; }

private:
  void thread_task(void);

  void iaq_init(void);
  void measure_iaq(void);

public:
  static constexpr uint8_t SGP30_8BIT_ADDRESS = 0x58 << 1;

private:
  I2C i2c_;
  Thread thread_;
  uint16_t id[3];
  uint16_t co2_ = 0;
  uint16_t voc_ = 0;
};

}; // namespace sensor

#endif
