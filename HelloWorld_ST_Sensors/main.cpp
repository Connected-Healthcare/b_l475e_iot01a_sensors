/**
 ******************************************************************************
 * @file    main.cpp
 * @author  CLab
 * @version V1.0.0
 * @date    5-September-2017
 * @brief   Simple Example application for using X_NUCLEO_IKS01A2
 *          MEMS Inertial & Environmental Sensor Nucleo expansion and
 *          B-L475E-IOT01A2 boards.
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT(c) 2017 STMicroelectronics</center></h2>
 *
 * Redistribution and use in source and binary forms, with or without
 *modification, are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright
 *notice, this list of conditions and the following disclaimer in the
 *documentation and/or other materials provided with the distribution.
 *   3. Neither the name of STMicroelectronics nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 *ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 *LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 *CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 *SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 *INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 *CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 *ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */

/* Includes */
#include "i2c_slave.hpp"
#include "internal_sensors.hpp"
#include "mbed.h"
#include "spec_co.hpp"
#include "gps.hpp"
#include "heartbeat.hpp"

// Sending data over BT
#include "hc05.hpp"

// Sending data over TCP or UDP
#include "internet.h"

#define DEBUG_PRINT 1

#if DEBUG_PRINT
#define debugPrintf(...) printf(__VA_ARGS__)
#else
#define debugPrintf(...)
#endif

static spec::CarbonMonoxide co(PA_0, PA_1);                 // UART4
static gps::adafruit_PA6H gps_obj(PC_4, PC_5, 9600);        // UART3
static i2c_slave::SlaveCommunication slave(PC_1, PC_0, co); // I2C1
static bt::hc05 btserial(PA_2, PA_3, 9600);                 // UART2
static heartbeat::sparkfun_MAX32664 hb_obj(PB_9, PB_8);     // I2C3

volatile bool is_recv = false;
char btserial_data[200];

void btserial_process(const char *line)
{
  memset(btserial_data, 0, sizeof(btserial_data));
  strcpy(btserial_data, line);
  is_recv = true;
}

volatile bool is_gps_recv = false;

gps::gps_coordinates_s gps_coordinates_data;

void gps_get_line(void)
{
  gps_coordinates_data = gps::get_gps_coordinates();
  is_gps_recv = true;
}

int main()
{
  internal_sensor::init_sensor();
  co.initialize();
  slave.init_thread();
  btserial.register_process_func(btserial_process);
  gps_obj.register_func(gps_get_line);
  hb_obj.hb_start();

  // NOTE, Always init this after the sensors have been initialized
  bool connected_to_internet = internet::connect_as_tcp();
  // Add all your sensor classes here
  internet::sensors_t sensors;
  sensors.co = &co;

  // Start the thread
  Thread internet_thread;
  if (connected_to_internet)
  {
    printf("Started Internet Thread\r\n");
    internet_thread.start(callback(internet::send_sensor_data, &sensors));
  }

  debugPrintf("\r\n--- Starting new run ---\r\n\r\n");
  ThisThread::sleep_for(1000);
  char buffer[200] = {0};

  while (1)
  {

    // Receive BT data from Android App to board
    if (is_recv)
    {
      is_recv = false;
      printf("btrecv: %s\r\n", btserial_data);
    }

    debugPrintf("ST B-L475E-IOT01A - I2C Slave\r\n");

    // Internal Sensor data
    internal_sensor::update_sensor_data();
    const internal_sensor::data_s &data = internal_sensor::get_sensor_data();
    debugPrintf("HTS221: Temperature(C):          %.2f     | Humidity: %.2f\r\n",
                data.hts221_temperature, data.hts221_humidity);
    debugPrintf("LPS22HB: Temperature(C):         %.2f     | Pressure(mbar): %.2f\r\n",
                data.lps22hb_temperature, data.lps22hb_pressure);
    debugPrintf("LIS3MDL: Magnetic Field(mgauss): X: %6ld | Y: %6ld | Z: %6ld\r\n",
                data.magnetometer_axes[0], data.magnetometer_axes[1],
                data.magnetometer_axes[2]);
    debugPrintf("LSM6DSL: Acceleration(mg):       X: %6ld | Y: %6ld | Z: %6ld\r\n",
                data.acceleration_axes[0], data.acceleration_axes[1],
                data.acceleration_axes[2]);
    debugPrintf("LSM6DSL: Gyroscope(mdps):        X: %6ld | Y: %6ld | Z: %6ld\r\n",
                data.gyroscope_axes[0], data.gyroscope_axes[1],
                data.gyroscope_axes[2]);
    debugPrintf("VL53L0X: Time of Flight(mm): %7ld\r\n", data.distance);

    // CO Sensor
    uint32_t spec_co_gas_conc = co.get_gas_concentration();
    int16_t spec_co_temp = co.get_temperature();
    uint16_t spec_co_rel_humidity = co.get_relative_humidity();

    debugPrintf("IOT_CO_1000_CO:                  Gas Conc.(PPM): %4lu | Temperature(C): %d | "
                "Relative Humidity: %u\r\n",
                spec_co_gas_conc, spec_co_temp,
                spec_co_rel_humidity);

    // GPS
    if (is_gps_recv == true)
    {
      debugPrintf("AP6H GPS:                        Latitude(deg): %f | Longitude(deg): %f\r\n", gps_coordinates_data.latitude, gps_coordinates_data.longitude);
      is_gps_recv = false;
    }

    // Heartbeat Sensor
    heartbeat::bioData hb_data = heartbeat::get_hb_data();
    debugPrintf("MAX32664/MAX30101 Heartbeat:     Heart Rate(bpm): %3d | Confidence: %3d    | Oxygen(%%): %d | Status: %d\r\n", hb_data.heartRate, hb_data.confidence, hb_data.oxygen, hb_data.status);

    debugPrintf("-----\r\n");

    // Send BT data from board to Android App
    sprintf(buffer,
            "%.2f,%.2f,%.2f,%.2f,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,"
            "%ld,%ld,%d,%d,%lu,%d,%u,%.2f,%.2f\r\n",
            data.hts221_temperature, data.hts221_humidity,
            data.lps22hb_temperature, data.lps22hb_pressure,
            data.magnetometer_axes[0], data.magnetometer_axes[1],
            data.magnetometer_axes[2], data.acceleration_axes[0],
            data.acceleration_axes[1], data.acceleration_axes[2],
            data.gyroscope_axes[0], data.gyroscope_axes[1],
            data.gyroscope_axes[2], data.distance, hb_data.heartRate, hb_data.oxygen, spec_co_gas_conc, spec_co_temp, spec_co_rel_humidity, gps_coordinates_data.latitude, gps_coordinates_data.longitude);
    btserial.write(buffer);
    gps_coordinates_data.latitude = 0.0;
    gps_coordinates_data.longitude = 0.0;
    ThisThread::sleep_for(500);
  }
}
