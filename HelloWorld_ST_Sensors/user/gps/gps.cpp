#include "user/gps/gps.hpp"
#include "mbed.h"

#define DEBUG_PRINTF 0

#if DEBUG_PRINTF
#define debugPrintf(...) debugPrintf(__VA_ARGS__)
#else
#define debugPrintf(...)
#endif

namespace gps {
gps_coordinates_s &adafruit_PA6H::get_gps_coordinates(void) { return gps_data; }

void adafruit_PA6H::set_gps_coordinates(gps_coordinates_s gps_data) {
  this->gps_data = gps_data;
}

void adafruit_PA6H::gps_struct_init(void) {
  gps_coordinates_s temp_gps_data = {0};
  strcpy(temp_gps_data.lat_dir, "NA");
  temp_gps_data.latitude = 0.0;
  strcpy(temp_gps_data.lat_dir, "NA");
  temp_gps_data.longitude = 0.0;
  strcpy(temp_gps_data.validity, "NA");
  set_gps_coordinates(temp_gps_data);
}

uint8_t adafruit_PA6H::check_gprmc_line(char *line_buff) {
  uint16_t nmea_string_token_index = 0;
  char *temp_token = NULL;
  uint8_t gps_nmea_type = GPS_INVALID_NMEA;
  gps_coordinates_s temp_gps_data = {0};

  temp_token = strtok(line_buff, ",");

  if (strcmp(temp_token, "$GPRMC") == 0) {
    gps_nmea_type = GPS_GPRMC;
  }

  if (gps_nmea_type != GPS_INVALID_NMEA) {
    while (temp_token != NULL) {
      temp_token = strtok(NULL, ",");
      nmea_string_token_index++;
      switch (gps_nmea_type) {
      case GPS_GPRMC:
        switch (nmea_string_token_index) {
        case GPRMC_VALIDITY_INDEX:
          strcpy(temp_gps_data.validity, temp_token);
          // Only use NMEA strings that are GPRMC and have Navigation Receiver
          // Warning set to Active (A)
          if (strcmp(temp_token, "A") != 0) {
            gps_nmea_type = GPS_INVALID_NMEA;
          }
          break;
        case GPRMC_LATITUDE_INDEX:
          temp_gps_data.latitude = atof(temp_token);
          break;
        case GPRMC_LATITUDE_DIR_INDEX:
          strcpy(temp_gps_data.lat_dir, temp_token);
          break;
        case GPRMC_LONGITUDE_INDEX:
          temp_gps_data.longitude = atof(temp_token);
          break;
        case GPRMC_LONGITUDE_DIR_INDEX:
          strcpy(temp_gps_data.long_dir, temp_token);
          break;
        }
        break;
      }
    }
    if (strcmp(temp_gps_data.lat_dir, "S") == 0) {
      temp_gps_data.latitude = ((float)(-1.0) * temp_gps_data.latitude);
    }
    if (strcmp(temp_gps_data.long_dir, "W") == 0) {
      temp_gps_data.longitude = ((float)(-1.0) * temp_gps_data.longitude);
    }
    set_gps_coordinates(temp_gps_data);
  }

  return gps_nmea_type;
}

void adafruit_PA6H::extract_gprmc_line(char *gps_buff) {
  uint8_t gps_nmea_type = GPS_INVALID_NMEA;
  char valid_gps_string[100] = {0};
  uint16_t counter = 0;
  for (uint16_t i = 0; i < UART_BUF_LEN; i++) {
    if (gps_buff[i] == '$') {
      while (gps_buff[i] != '\n' && i < UART_BUF_LEN) {
        valid_gps_string[counter] = gps_buff[i];
        i++;
        counter++;
      }
      if (i < UART_BUF_LEN) {
        gps_nmea_type = check_gprmc_line(valid_gps_string);
        if (gps_nmea_type != GPS_INVALID_NMEA) {
          break;
        }
      }
      counter = 0;
      memset(valid_gps_string, 0, strlen(valid_gps_string));
    }
  }
}

void adafruit_PA6H::gps_interrupt_cb(void) {
  int data = gps_uart.getc();
  curr_line_buffer[index] = data;
  if (index != UART_BUF_LEN) {
    index++;
  }

  if (data == '\n') {
    extract_gprmc_line(curr_line_buffer);
    memset(curr_line_buffer, 0, UART_BUF_LEN);
    index = 0;
  }
}
} // namespace gps
