#ifndef USER_HC05_H_
#define USER_HC05_H_

#include "mbed.h"

namespace bt {

class hc05 {
 private:
  static constexpr uint8_t kBufferSize = 200;

 public:
  hc05(PinName tx, PinName rx, uint32_t baud) : bt_uart_(tx, rx) {
    bt_uart_.baud(baud);
    bt_uart_.attach(callback(this, &hc05::interrupt_cb));
  }

  void write(const char* data);
  void register_process_func(void (*process)(const char* newline));

 private:
  void interrupt_cb();

 private:
  RawSerial bt_uart_;
  char BUF_[kBufferSize];
  size_t counter_;

  void (*process_func_)(const char* newline);
};

}  // namespace bt
#endif
