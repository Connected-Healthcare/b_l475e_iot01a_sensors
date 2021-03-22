#ifndef USER_HC05_H_
#define USER_HC05_H_

#include "mbed.h"

namespace bt
{

  class hc05
  {
  private:
    static constexpr uint8_t kBufferSize = 200;

  public:
    hc05(PinName tx, PinName rx, uint32_t baud) : bt_uart_(tx, rx, baud)
    {
      bt_uart_.attach(callback(this, &hc05::interrupt_cb));
    }

    /**
   * @brief Write data from the microcontroller to client
   * NOTE: Unexpected behaviour if C String is not NULL terminated
   *
   * @param data Null terminated C String
   */
    void write(const char *data);

    /**
   * @brief Write data from microcontroller to client
   * NOTE: Safer alternative since write is sz bounded
   *
   * @param data Null terminated C String
   * @param bytes_to_write Number of bytes to write
   */
    void write(const char *data, size_t bytes_to_write);
    void register_process_func(void (*process)(const char *newline));

  private:
    void interrupt_cb();

  private:
    RawSerial bt_uart_;
    char read_buf_[kBufferSize];
    size_t counter_;

    void (*process_func_)(const char *newline);
  };

} // namespace bt
#endif
