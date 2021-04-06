#include "user/internal/internal_sensors.hpp"

#include "mbed.h"

#include "HTS221Sensor.h"
#include "LPS22HBSensor.h"
#include "LSM6DSLSensor.h"

#include "lis3mdl_class.h"

#include "VL53L0X.h"

namespace internal_sensor {

static DevI2C devI2c(PB_11, PB_10);
static HTS221Sensor hum_temp(&devI2c);
static LPS22HBSensor press_temp(&devI2c);
static LSM6DSLSensor acc_gyro(&devI2c, LSM6DSL_ACC_GYRO_I2C_ADDRESS_LOW, PD_11);
static LIS3MDL magnetometer(&devI2c);

static DigitalOut shutdown_pin(PC_6);
static VL53L0X range(&devI2c, &shutdown_pin, PC_7);

static constexpr uint32_t DEBUG = 0;

static data_s sensor_data;

void init_sensor(void) {
  hum_temp.init(NULL);
  press_temp.init(NULL);
  magnetometer.init(NULL);
  acc_gyro.init(NULL);
  range.init_sensor(VL53L0X_DEFAULT_ADDRESS);

  hum_temp.enable();
  press_temp.enable();
  acc_gyro.enable_x();
  acc_gyro.enable_g();

#if DEBUG
  uint8_t id;
  hum_temp.read_id(&id);
  printf("HTS221  humidity & temperature    = 0x%X\r\n", id);
  press_temp.read_id(&id);
  printf("LPS22HB pressure & temperature    = 0x%X\r\n", id);
  magnetometer.read_id(&id);
  printf("LIS3MDL magnetometer              = 0x%X\r\n", id);
  acc_gyro.read_id(&id);
  printf("LSM6DSL accelerometer & gyroscope = 0x%X\r\n", id);
#endif
}

void update_sensor_data(void) {
  hum_temp.get_temperature(&sensor_data.hts221_temperature);
  hum_temp.get_humidity(&sensor_data.hts221_humidity);

  press_temp.get_temperature(&sensor_data.lps22hb_temperature);
  press_temp.get_pressure(&sensor_data.lps22hb_pressure);

  magnetometer.get_m_axes(sensor_data.magnetometer_axes);

  acc_gyro.get_x_axes(sensor_data.acceleration_axes);

  acc_gyro.get_g_axes(sensor_data.gyroscope_axes);

  int status = range.get_distance(&sensor_data.distance);
  if (status == VL53L0X_ERROR_NONE) {
    // sensor_data.distance = 0;
    // TODO, Do some processing
  }
}

const data_s &get_sensor_data(void) { return sensor_data; }

} // namespace internal_sensor
