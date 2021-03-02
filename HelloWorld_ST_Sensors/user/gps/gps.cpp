#include "gps.hpp"

namespace gps_ns
{
    gps_c::gps_coordinates_s gps_c::get_gps_coordinates(void)
    {
        return gps_coordinates_data;
    }

    void gps_c::gps_struct_init(void)
    {
        strcpy(gps_coordinates_data.lat_dir, "NA");
        strcpy(gps_coordinates_data.latitude, "0.0");
        strcpy(gps_coordinates_data.lat_dir, "NA");
        strcpy(gps_coordinates_data.longitude, "0.0");
        strcpy(gps_coordinates_data.validity, "NA");
    }

    uint8_t gps_c::check_gprmc_gpgga_line(char *line_buff)
    {
        uint16_t nmea_string_token_index = 0;
        char *temp_token = NULL;
        uint8_t gps_nmea_type = GPS_INVALID_NMEA;

        temp_token = strtok(line_buff, ",");

        if (strcmp(temp_token, "$GPRMC") == 0)
        {
            gps_nmea_type = GPS_GPRMC;
        }

        // Not processing GPGGA since it does not have an Active/Valid field to determine the validity of NMEA string
        // else if (strcmp(temp_token, "$GPGGA") == 0)
        // {
        //     gps_nmea_type = GPS_GPGGA;
        // }

        if (gps_nmea_type != GPS_INVALID_NMEA)
        {
            while (temp_token != NULL)
            {
                temp_token = strtok(NULL, ",");
                nmea_string_token_index++;
                switch (gps_nmea_type)
                {
                case GPS_GPGGA:
                    switch (nmea_string_token_index)
                    {
                    case GPGGA_LATITUDE_INDEX:
                        strcpy(gps_coordinates_data.latitude, temp_token);
                        break;
                    case GPGGA_LATITUDE_DIR_INDEX:
                        strcpy(gps_coordinates_data.lat_dir, temp_token);
                        break;
                    case GPGGA_LONGITUDE_INDEX:
                        strcpy(gps_coordinates_data.longitude, temp_token);
                        break;
                    case GPGGA_LONGITUDE_DIR_INDEX:
                        strcpy(gps_coordinates_data.long_dir, temp_token);
                        break;
                    }
                    break;
                case GPS_GPRMC:
                    switch (nmea_string_token_index)
                    {
                    case GPRMC_VALIDITY_INDEX:
                        strcpy(gps_coordinates_data.validity, temp_token);
                        // Consider NMEA strings that are GPRMC and Active Navigation Receiver Warning
                        if (strcmp(temp_token, "A") != 0)
                        {
                            gps_nmea_type = GPS_INVALID_NMEA;
                        }
                        break;
                    case GPRMC_LATITUDE_INDEX:
                        strcpy(gps_coordinates_data.latitude, temp_token);
                        break;
                    case GPRMC_LATITUDE_DIR_INDEX:
                        strcpy(gps_coordinates_data.lat_dir, temp_token);
                        break;
                    case GPRMC_LONGITUDE_INDEX:
                        strcpy(gps_coordinates_data.longitude, temp_token);
                        break;
                    case GPRMC_LONGITUDE_DIR_INDEX:
                        strcpy(gps_coordinates_data.long_dir, temp_token);
                        break;
                    }
                    break;
                }
            }
        }
        return gps_nmea_type;
    }

    uint8_t gps_c::extract_gpgga_or_gprmc_line(char *gps_buff)
    {
        uint8_t gps_nmea_type = GPS_INVALID_NMEA;
        char valid_gps_string[100] = {0};
        uint16_t counter = 0;
        for (uint16_t i = 0; i < UART_BUF_LEN; i++)
        {
            if (gps_buff[i] == '$')
            {
                while (gps_buff[i] != '\n' && i < UART_BUF_LEN)
                {
                    valid_gps_string[counter] = gps_buff[i];
                    i++;
                    counter++;
                }
                if (i < UART_BUF_LEN)
                {
                    gps_nmea_type = check_gprmc_gpgga_line(valid_gps_string);
                    if (gps_nmea_type != GPS_INVALID_NMEA)
                    {
                        break;
                    }
                }
                counter = 0;
                memset(valid_gps_string, 0, strlen(valid_gps_string));
            }
        }
        return gps_nmea_type;
    }

    void gps_c::register_func(void (*external_fn)(const char *newline))
    {
        gps_process_fn_ptr = external_fn;
    }

    void gps_c::gps_interrupt_cb()
    {
        int data = gps_uart.getc();
        curr_line_buffer[index] = data;
        if (index != UART_BUF_LEN)
        {
            index++;
        }
        if (index == UART_BUF_LEN)
        {
            uint8_t gps_nmea_type = extract_gpgga_or_gprmc_line(curr_line_buffer);
            if ((GPS_INVALID_NMEA != gps_nmea_type) && (gps_process_fn_ptr != NULL))
            {
                char buffer_coordinate[128] = {0};
                sprintf(buffer_coordinate, "Latitude: %s %s; Longitude: %s %s", gps_coordinates_data.latitude, gps_coordinates_data.lat_dir, gps_coordinates_data.longitude, gps_coordinates_data.long_dir);
                gps_process_fn_ptr(buffer_coordinate);
            }
            memset(curr_line_buffer, 0, UART_BUF_LEN);
            index = 0;
            // memset(&gps_coordinates_data, 0, sizeof(gps_coordinates_s));
            gps_struct_init();
        }
    }
}
