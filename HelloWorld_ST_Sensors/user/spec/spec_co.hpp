#ifndef USER_SPEC_CO_H_
#define USER_SPEC_CO_H_

#include "mbed.h"

namespace spec {

class CarbonMonoxide {
private:
  static constexpr size_t kBufferSize = 200;

public:
  CarbonMonoxide(PinName tx, PinName rx) : co_uart_(tx, rx), counter_(0) {
    memset(BUF_, 0, kBufferSize);
    memset(PUB_BUF_, 0, kBufferSize);
  }

  void initialize();

  uint32_t get_gas_concentration();
  int16_t get_temperature();
  uint16_t get_relative_humidity();

private:
  void thread_start();
  void interrupt_cb();

public:
  char PUB_BUF_[kBufferSize];

private:
  RawSerial co_uart_;
  Thread co_thread_;
  char BUF_[kBufferSize];
  size_t counter_;

  volatile bool recv = false;

  uint32_t gas_concentration_;
  int16_t temperature_;
  uint16_t relative_humidity_;
};

} // namespace spec

#endif
