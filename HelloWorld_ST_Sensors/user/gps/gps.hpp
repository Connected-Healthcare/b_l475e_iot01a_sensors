#ifndef USER_GPS_H_
#define USER_GPS_H_

#include "mbed.h"
#include <string.h>

namespace gps {
struct gps_coordinates_s {
  float latitude;
  char lat_dir[3];
  float longitude;
  char long_dir[3];
  char validity[4];
};

enum GPS_NMEA_STRING_TYPE { GPS_INVALID_NMEA, GPS_GPGGA, GPS_GPRMC };

enum GPRMC_NMEA_STRING_INDEX_DEFINITION {
  GPRMC_VALIDITY_INDEX = 2,
  GPRMC_LATITUDE_INDEX = 3,
  GPRMC_LATITUDE_DIR_INDEX = 4,
  GPRMC_LONGITUDE_INDEX = 5,
  GPRMC_LONGITUDE_DIR_INDEX = 6,
};

class adafruit_PA6H {
private:
  static constexpr uint32_t UART_BUF_LEN = 128;
  RawSerial gps_uart;
  uint32_t index;

  uint8_t check_gprmc_line(char *check_for_rmc_gga_in_this_str);
  void extract_gprmc_line(char *gps_buff);
  char curr_line_buffer[UART_BUF_LEN];

  void gps_interrupt_cb(void);
  void gps_struct_init(void);
  gps_coordinates_s gps_data;

public:
  adafruit_PA6H(PinName tx, PinName rx, uint32_t baud)
      : gps_uart(tx, rx, baud) {
    gps_uart.attach(callback(this, &adafruit_PA6H::gps_interrupt_cb));
  }

  const gps_coordinates_s &get_gps_coordinates(void);

  void set_gps_coordinates(gps_coordinates_s gps_data);
};
} // namespace gps
#endif
