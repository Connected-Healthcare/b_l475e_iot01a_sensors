#include "user/spec/spec_co.hpp"

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
    process_raw_co_data();
    memset(BUF_, 0, kBufferSize);
    counter_ = 0;
  }
}

void CarbonMonoxide::process_raw_co_data() {
  constexpr char kDelim[] = ",";
  // id
  char *token = strtok(BUF_, kDelim);

  // convert to co_ppb
  token = strtok(NULL, kDelim);
  gas_concentration_ = strtoul(token, NULL, 0);

  // convert to temperature
  token = strtok(NULL, kDelim);
  temperature_ = atoi(token);

  // convert to humidity
  token = strtok(NULL, kDelim);
  relative_humidity_ = atoi(token);
}

} // namespace spec
