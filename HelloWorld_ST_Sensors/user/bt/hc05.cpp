#include "hc05.hpp"

namespace bt {

void hc05::write(const char* data) {
  while (*data != 0) {
    bt_uart_.putc(*data);
    data++;
  }
}

void hc05::write(const char* data, size_t bytes_to_write) {
  for (size_t i = 0; i < bytes_to_write; i++) {
    bt_uart_.putc(data[i]);
  }
}

void hc05::register_process_func(void (*process)(const char* newline)) {
  process_func_ = process;
}

void hc05::interrupt_cb() {
  int data = bt_uart_.getc();
  read_buf_[counter_] = data;
  if (counter_ != kBufferSize) {
    counter_++;
  }

  if (data == '\n') {
    if (process_func_ != nullptr) {
      process_func_(read_buf_);
    }
    memset(read_buf_, 0, kBufferSize);
    counter_ = 0;
  }
}

}  // namespace bt
