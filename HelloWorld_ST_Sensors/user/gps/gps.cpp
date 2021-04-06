#include "user/gps/gps.hpp"

namespace gps {
gps_coordinates_s get_gps_coordinates(void) { return gps_coordinates_data; }

void adafruit_PA6H::gps_struct_init(void) {
  strcpy(gps_coordinates_data.lat_dir, "NA");
  gps_coordinates_data.latitude = 0.0;
  strcpy(gps_coordinates_data.lat_dir, "NA");
  gps_coordinates_data.longitude = 0.0;
  strcpy(gps_coordinates_data.validity, "NA");
}

uint8_t adafruit_PA6H::check_gprmc_line(char *line_buff) {
  uint16_t nmea_string_token_index = 0;
  char *temp_token = NULL;
  uint8_t gps_nmea_type = GPS_INVALID_NMEA;

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
          strcpy(gps_coordinates_data.validity, temp_token);
          // Only use NMEA strings that are GPRMC and have Navigation Receiver
          // Warning set to Active (A)
          if (strcmp(temp_token, "A") != 0) {
            gps_nmea_type = GPS_INVALID_NMEA;
          }
          break;
        case GPRMC_LATITUDE_INDEX:
          gps_coordinates_data.latitude = atof(temp_token);
          break;
        case GPRMC_LATITUDE_DIR_INDEX:
          strcpy(gps_coordinates_data.lat_dir, temp_token);
          break;
        case GPRMC_LONGITUDE_INDEX:
          gps_coordinates_data.longitude = atof(temp_token);
          break;
        case GPRMC_LONGITUDE_DIR_INDEX:
          strcpy(gps_coordinates_data.long_dir, temp_token);
          break;
        }
        break;
      }
    }
  }

  if (strcmp(gps_coordinates_data.lat_dir, "S") == 0) {
    gps_coordinates_data.latitude =
        ((float)(-1.0) * gps_coordinates_data.latitude);
  }
  if (strcmp(gps_coordinates_data.long_dir, "W") == 0) {
    gps_coordinates_data.longitude =
        ((float)(-1.0) * gps_coordinates_data.longitude);
  }
  return gps_nmea_type;
}

uint8_t adafruit_PA6H::extract_gprmc_line(char *gps_buff) {
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
  return gps_nmea_type;
}

void adafruit_PA6H::register_func(void (*external_fn)(void)) {
  gps_process_fn_ptr = external_fn;
}

void adafruit_PA6H::gps_interrupt_cb() {
  int data = gps_uart.getc();
  curr_line_buffer[index] = data;
  if (index != UART_BUF_LEN) {
    index++;
  }

  if (data == '\n') {
    uint8_t gps_nmea_type = extract_gprmc_line(curr_line_buffer);
    if ((GPS_INVALID_NMEA != gps_nmea_type) && (gps_process_fn_ptr != NULL)) {
      gps_process_fn_ptr();
    }
    memset(curr_line_buffer, 0, UART_BUF_LEN);
    index = 0;
  }
}
} // namespace gps
