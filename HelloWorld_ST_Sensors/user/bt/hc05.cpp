#include "hc05.hpp"

namespace bt {

void hc05::write(const char* data) {
  while (*data != 0) {
    bt_uart_.putc(*data);
    data++;
  }
}

void hc05::register_process_func(void (*process)(const char* newline)) {
  process_func_ = process;
}

void hc05::interrupt_cb() {
  int data = bt_uart_.getc();
  BUF_[counter_] = data;
  if (counter_ != kBufferSize) {
    counter_++;
  }

  if (data == '\n') {
    if (process_func_ != nullptr) {
      process_func_(BUF_);
    }
    memset(BUF_, 0, kBufferSize);
    counter_ = 0;
  }
}

}  // namespace bt
