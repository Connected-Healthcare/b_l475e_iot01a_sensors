#ifndef USER_SPEC_CO_H_
#define USER_SPEC_CO_H_

#include "mbed.h"

namespace spec {

class CarbonMonoxide {
private:
  static constexpr size_t kBufferSize = 200;

public:
  CarbonMonoxide(PinName tx, PinName rx) : co_uart_(tx, rx) {}

  void initialize();

  uint32_t get_gas_concentration();
  int16_t get_temperature();
  uint16_t get_relative_humidity();

private:
  void thread_start();

  void interrupt_cb();
  void process_raw_co_data();

private:
  RawSerial co_uart_;
  Thread co_thread_;

  char BUF_[kBufferSize] = {0};
  size_t counter_ = 0;

  volatile bool recv = false;

  uint32_t gas_concentration_ = 0;
  int16_t temperature_ = 0;
  uint16_t relative_humidity_ = 0;
};

} // namespace spec

#endif
