#ifndef USER_GPS_H_
#define USER_GPS_H_

#include "mbed.h"
#include <string.h>

namespace gps_ns
{
    class gps_c
    {
    private:
        static constexpr uint32_t UART_BUF_LEN = 400;
        RawSerial gps_uart;
        uint32_t index;

        uint8_t check_gprmc_gpgga_line(char *check_for_rmc_gga_in_this_str);
        uint8_t extract_gpgga_or_gprmc_line(char *gps_buff);

        void gps_interrupt_cb(void);
        void gps_struct_init(void);
        void (*gps_process_fn_ptr)(const char *newline);

    public:
        gps_c(PinName tx, PinName rx, uint32_t baud) : gps_uart(tx, rx)
        {
            gps_uart.baud(baud);
            gps_uart.attach(callback(this, &gps_c::gps_interrupt_cb));
        }

        typedef struct gps_data
        {
            char latitude[15];
            char longitude[15];
            char lat_dir[3];
            char long_dir[3];
            char validity[4];
        } gps_coordinates_s;

        gps_coordinates_s gps_coordinates_data;
        gps_coordinates_s get_gps_coordinates(void);

        enum GPS_NMEA_STRING_TYPE
        {
            GPS_INVALID_NMEA,
            GPS_GPGGA,
            GPS_GPRMC
        };

        enum GPS_NMEA_STRING_INDEX_DEFINITION
        {
            GPGGA_LATITUDE_INDEX = 2,
            GPGGA_LATITUDE_DIR_INDEX = 3,
            GPGGA_LONGITUDE_INDEX = 4,
            GPGGA_LONGITUDE_DIR_INDEX = 5,
            GPRMC_VALIDITY_INDEX = 2,
            GPRMC_LATITUDE_INDEX = 3,
            GPRMC_LATITUDE_DIR_INDEX = 4,
            GPRMC_LONGITUDE_INDEX = 5,
            GPRMC_LONGITUDE_DIR_INDEX = 6,
        };

        char curr_line_buffer[UART_BUF_LEN];
        void register_func(void (*external_fn)(const char *newline));
    };
}
#endif
