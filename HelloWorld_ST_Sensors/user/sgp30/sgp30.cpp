#include "user/sgp30/sgp30.hpp"

namespace sensor
{
  void SGP30::start(void)
  {
    iaq_init();
    thread_.start(callback(this, &SGP30::thread_task));
  }

  // PRIVATE FUNCTIONS

  void SGP30::thread_task(void)
  {
    measure_iaq();
    while (1)
    {
      // measure_iaq sleeps for 12 ms
      ThisThread::sleep_for(1000 - 12);
      measure_iaq();

      // TODO, Do some error checking
    }
  }

  void SGP30::iaq_init(void)
  {
    constexpr char cmd[2] = {0x20, 0x03};
    i2c_.write(SGP30_8BIT_ADDRESS, cmd, 2);
    ThisThread::sleep_for(10);
  }

  void SGP30::measure_iaq(void)
  {
    constexpr char cmd[2] = {0x20, 0x08};
    i2c_.write(SGP30_8BIT_ADDRESS, cmd, 2);
    ThisThread::sleep_for(12);

    char recv[6] = {0};
    i2c_.read(SGP30_8BIT_ADDRESS, recv, 6);
    co2_ = (recv[0] << 8) | recv[1]; // recv[2] is CRC
    voc_ = (recv[3] << 8) | recv[4]; // recv[5] is CRC
  }

} // namespace sensor
