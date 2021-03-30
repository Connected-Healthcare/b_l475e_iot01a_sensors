#include "internet.h"

#include "mbed.h"

#include "internal_sensors.hpp"
#include "user/heartbeat/heartbeat_logic.hpp"
#include "user/gps/gps.hpp"

#include "access_point.h"
#include "wifi.h"

static uint8_t mac_address[6];
static uint8_t ip_address[4];

constexpr uint16_t kPort = REMOTE_ADDRESS_PORT;
constexpr uint32_t kSocket = 0;
constexpr uint32_t kSendTimeout = 1000;
constexpr uint32_t kThreadSleep = 1000; // milliseconds

// State variables
static uint8_t send_data[1000] = {0};

namespace internet
{

  static void update_co_data(spec::CarbonMonoxide *co);
  static void update_internal_data();

  bool connect_as_tcp(void)
  {
    WIFI_Status_t wifi_status;

    wifi_status = WIFI_Init();
    if (wifi_status != WIFI_STATUS_OK)
    {
      printf("WIFI Not INIT\r\n");
      return false;
    }

    wifi_status = WIFI_GetMAC_Address(mac_address);
    if (wifi_status != WIFI_STATUS_OK)
    {
      printf("WIFI could not obtain MAC Address\r\n");
      return false;
    }
    printf("mac_address: %X:%X:%X:%X:%X:%X\n", mac_address[0], mac_address[1],
           mac_address[2], mac_address[3], mac_address[4], mac_address[5]);

    wifi_status = WIFI_Connect(ACCESS_POINT_SSID, ACCESS_POINT_PASSWORD,
                               ACCESS_POINT_ENCRYPTION);
    if (wifi_status != WIFI_STATUS_OK)
    {
      printf("WIFI could not connect\r\n");
      return false;
    }

    wifi_status = WIFI_GetIP_Address(ip_address);
    if (wifi_status != WIFI_STATUS_OK)
    {
      printf("WIFI get IP Address\r\n");
      return false;
    }
    printf("get_ip_address: %d:%d:%d:%d\r\n", ip_address[0], ip_address[1],
           ip_address[2], ip_address[3]);

    //  TCP Connection
    uint8_t remote_ip[4] = {REMOTE_ADDRESS_CONNECTION};
    wifi_status = WIFI_OpenClientConnection(kSocket, WIFI_TCP_PROTOCOL,
                                            "TCP_CLIENT", remote_ip, kPort, 0);
    if (wifi_status != WIFI_STATUS_OK)
    {
      printf("WIFI could not open client Connection\r\n");
      return false;
    }

    return true;
  }

  void send_sensor_data(sensors_t *sensors)
  {
    uint16_t sent_datalen = 0;
    while (1)
    {
      update_co_data(sensors->co);
      update_internal_data();

      WIFI_SendData(kSocket, send_data, strlen((char *)send_data), &sent_datalen,
                    kSendTimeout);
      memset(send_data, 0, sizeof(send_data));
      ThisThread::sleep_for(kThreadSleep);
    }
  }

  static void update_co_data(spec::CarbonMonoxide *co)
  {
    uint8_t buffer[10 + 5 + 5 + 3 + 1] = {0};
    sprintf((char *)buffer, "%lu,%d,%u,", co->get_gas_concentration(),
            co->get_temperature(), co->get_relative_humidity());
    strcat((char *)send_data, (const char *)buffer);
  }

  static void update_internal_data()
  {
    uint8_t buffer[8 * 4 + 10 * 10 + 10 + 1] = {0};

    const internal_sensor::data_s &data = internal_sensor::get_sensor_data();
    hb_sensor::bioData hb_data = hb_sensor::get_hb_data();
    gps::gps_coordinates_s gps_data = gps::get_gps_coordinates();

    sprintf((char *)buffer,
            "%.2f,%.2f,%.2f,%.2f,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%.2f,%.2f",
            data.hts221_temperature, data.hts221_humidity,
            data.lps22hb_temperature, data.lps22hb_pressure,
            data.magnetometer_axes[0], data.magnetometer_axes[1],
            data.magnetometer_axes[2], data.acceleration_axes[0],
            data.acceleration_axes[1], data.acceleration_axes[2],
            data.gyroscope_axes[0], data.gyroscope_axes[1],
            data.gyroscope_axes[2], data.distance, hb_data.heartRate, hb_data.oxygen,
            gps_data.latitude, gps_data.longitude);
    strcat((char *)send_data, (const char *)buffer);
  }

} // namespace internet
