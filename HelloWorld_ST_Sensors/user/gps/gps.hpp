#ifndef USER_GPS_H_
#define USER_GPS_H_

#include "mbed.h"
#include <string.h>

namespace gps {
typedef struct gps_data {
  float latitude;
  char lat_dir[3];
  float longitude;
  char long_dir[3];
  char validity[4];
} gps_coordinates_s;

enum GPS_NMEA_STRING_TYPE { GPS_INVALID_NMEA, GPS_GPGGA, GPS_GPRMC };

enum GPRMC_NMEA_STRING_INDEX_DEFINITION {
  GPRMC_VALIDITY_INDEX = 2,
  GPRMC_LATITUDE_INDEX = 3,
  GPRMC_LATITUDE_DIR_INDEX = 4,
  GPRMC_LONGITUDE_INDEX = 5,
  GPRMC_LONGITUDE_DIR_INDEX = 6,
};

static gps_coordinates_s gps_coordinates_data;

gps_coordinates_s get_gps_coordinates(void);

class adafruit_PA6H {
private:
  static constexpr uint32_t UART_BUF_LEN = 128;
  RawSerial gps_uart;
  uint32_t index;

  uint8_t check_gprmc_line(char *check_for_rmc_gga_in_this_str);
  uint8_t extract_gprmc_line(char *gps_buff);
  char curr_line_buffer[UART_BUF_LEN];

  void gps_interrupt_cb(void);
  void gps_struct_init(void);
  void (*gps_process_fn_ptr)();

public:
  adafruit_PA6H(PinName tx, PinName rx, uint32_t baud)
      : gps_uart(tx, rx, baud) {
    gps_uart.attach(callback(this, &adafruit_PA6H::gps_interrupt_cb));
  }

  void register_func(void (*external_fn)(void));
};
} // namespace gps
#endif
