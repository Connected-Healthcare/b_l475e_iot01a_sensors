#include "spec_co.hpp"

namespace spec {

uint32_t CarbonMonoxide::get_gas_concentration() { return gas_concentration_; }
int16_t CarbonMonoxide::get_temperature() { return temperature_; }
uint16_t CarbonMonoxide::get_relative_humidity() { return relative_humidity_; }

void CarbonMonoxide::initialize() {
  co_uart_.attach(callback(this, &CarbonMonoxide::interrupt_cb));
  co_thread_.start(callback(this, &CarbonMonoxide::thread_start));
}

// Private
void CarbonMonoxide::thread_start() {
  while (recv == false) {
    co_uart_.putc('c');
    co_uart_.putc('c');
    co_uart_.putc('\n');
    ThisThread::sleep_for(1000);
  }
}

void CarbonMonoxide::interrupt_cb() {
  if (recv == false) {
    recv = true;
  }

  int data = co_uart_.getc();
  BUF_[counter_] = data;
  if (counter_ != kBufferSize) {
    counter_++;
  }

  if (data == '\n') {
    // TODO, Proces this string here
    // Remove this later
    {
      memset(PUB_BUF_, 0, kBufferSize);
      strcpy(PUB_BUF_, BUF_);
    }

    memset(BUF_, 0, kBufferSize);
    counter_ = 0;
  }
}

} // namespace spec
