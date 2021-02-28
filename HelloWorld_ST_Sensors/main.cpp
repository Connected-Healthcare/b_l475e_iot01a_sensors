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
#include "mbed.h"

#include "sgp30.hpp"
#include "spec_co.hpp"
#include "internal_sensors.hpp"
#include "heartbeat_logic.hpp"

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
static spec::CarbonMonoxide co(PA_0, PA_1);                        // UART4
static sensor::SGP30 sgp30(PB_9, PB_8);                            // I2C1
static i2c_slave::SlaveCommunication slave(PC_1, PC_0, co, sgp30); // I2C3
static bt::hc05 btserial(PA_2, PA_3, 9600);                        // UART2
static hb_sensor::hb_sensor_class hb_obj(PB_9, PB_8);

volatile bool is_recv = false;
char btserial_data[200];

// NOTE, This runs inside a interrupt
// ! DO NOT RUN ANY BLOCKING FUNCTIONS HERE
// NOTE, This process cb has been exposed since we might need to perform some
// tasks depending on what we recv
void btserial_process(const char *line)
{
  memset(btserial_data, 0, sizeof(btserial_data));
  strcpy(btserial_data, line);
  is_recv = true;
}

int main()
{
  internal_sensor::init_sensor();
  co.initialize();
  sgp30.start();
  slave.init_thread();
  btserial.register_process_func(btserial_process);
  hb_obj.start();

  // NOTE, Always init this after the sensors have been initialized
  bool connected_to_internet = internet::connect_as_tcp();
  // Add all your sensor classes here
  internet::sensors_t sensors;
  sensors.sgp30 = &sgp30;
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
    // BTSerial Recv
    // ! NOTE, We can control the microcontroller depending on what data we recv
    // here
    if (is_recv)
    {
      is_recv = false;
      printf("btrecv: %s\r\n", btserial_data);
    }

    // Internal Sensor data
    internal_sensor::update_sensor_data();
    const internal_sensor::data_s &data = internal_sensor::get_sensor_data();
    debugPrintf("HTS221: [temp] %.2f C, [hum] %.2f\r\n",
                data.hts221_temperature, data.hts221_humidity);
    debugPrintf("LPS22HB: [temp] %.2f C, [press] %.2f mbar\r\n",
                data.lps22hb_temperature, data.lps22hb_pressure);
    debugPrintf("LIS3MDL [mag/mgauss]:    %6ld, %6ld, %6ld\r\n",
                data.magnetometer_axes[0], data.magnetometer_axes[1],
                data.magnetometer_axes[2]);
    debugPrintf("LSM6DSL [acc/mg]:        %6ld, %6ld, %6ld\r\n",
                data.acceleration_axes[0], data.acceleration_axes[1],
                data.acceleration_axes[2]);
    debugPrintf("LSM6DSL [gyro/mdps]:     %6ld, %6ld, %6ld\r\n",
                data.gyroscope_axes[0], data.gyroscope_axes[1],
                data.gyroscope_axes[2]);
    debugPrintf("VL53L0X [mm]:            %6ld\r\n", data.distance);

    // CO Sensor
    debugPrintf(
        "SPEC CO SENSOR: Gas Concentration (ppm) %lu Temperature (C) %d "
        "(Relative Humidity) %d\r\n",
        co.get_gas_concentration(), co.get_temperature(),
        co.get_relative_humidity());

    // SGP30 Sensor
    debugPrintf("SGP30: (co2) %d (voc) %d\r\n", sgp30.get_co2(),
                sgp30.get_voc());
    // Heartbeat Sensor
    debugPrintf("HEARTBEAT SENSOR:\r\n");
    debugPrintf("Heartrate: %d\r\n", hb_obj.body.heartRate);
    debugPrintf("Confidence: %d\r\n", hb_obj.body.confidence);
    debugPrintf("Oxygen: %d\r\n", hb_obj.body.oxygen);
    debugPrintf("Status: %d\r\n", hb_obj.body.status);

    debugPrintf("-----\r\n");
    sprintf(buffer,
            "%.2f,%.2f,%.2f,%.2f,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,"
            "%ld,%ld,%d,%d\r\n",
            data.hts221_temperature, data.hts221_humidity,
            data.lps22hb_temperature, data.lps22hb_pressure,
            data.magnetometer_axes[0], data.magnetometer_axes[1],
            data.magnetometer_axes[2], data.acceleration_axes[0],
            data.acceleration_axes[1], data.acceleration_axes[2],
            data.gyroscope_axes[0], data.gyroscope_axes[1],
            data.gyroscope_axes[2], data.distance, hb_obj.body.heartRate, hb_obj.body.oxygen);
    btserial.write(buffer);
    ThisThread::sleep_for(500);
  }
}
