#include "user/slave/i2c_slave.hpp"

#include "user/internal/internal_sensors.hpp"

#define DEBUG_PRINTF 0

#if DEBUG_PRINTF
#define debugPrintf(...) printf(__VA_ARGS__)
#else
#define debugPrintf(...)
#endif

void uint32_to_uint8_array(uint32_t data, uint8_t *arr) {
  arr[0] = (data >> (0 * 8)) & 0xFF;
  arr[1] = (data >> (1 * 8)) & 0xFF;
  arr[2] = (data >> (2 * 8)) & 0xFF;
  arr[3] = (data >> (3 * 8)) & 0xFF;
}

void uint64_to_uint8_array(uint64_t data, uint8_t *arr) {
  arr[0] = (data >> (0 * 8)) & 0xFF;
  arr[1] = (data >> (1 * 8)) & 0xFF;
  arr[2] = (data >> (2 * 8)) & 0xFF;
  arr[3] = (data >> (3 * 8)) & 0xFF;
  arr[4] = (data >> (4 * 8)) & 0xFF;
  arr[5] = (data >> (5 * 8)) & 0xFF;
  arr[6] = (data >> (6 * 8)) & 0xFF;
  arr[7] = (data >> (7 * 8)) & 0xFF;
}

void uint16_to_uint8_array(uint16_t data, uint8_t *arr) {
  arr[0] = (data >> (0 * 8)) & 0xFF;
  arr[1] = (data >> (1 * 8)) & 0xFF;
}

uint32_t convert_float_to_uint32_float_structure(float data) {
  uint32_t rdata = *(uint32_t *)&data;
  return rdata;
}

namespace i2c_slave {

void SlaveCommunication::init_thread() {
  slave_.address(I2C_SLAVE_8BIT_ADDRESS);
  thread_.start(callback(this, &SlaveCommunication::handle_data));
}

void SlaveCommunication::handle_data() {
  char rx_data = 0;
  while (1) {
    int i = slave_.receive();

    switch (i) {
    case I2CSlave::ReadAddressed:
      transmit_sensor_data(rx_data);
      break;
    case I2CSlave::WriteGeneral:
      debugPrintf("Write General\r\n");
      break;
    case I2CSlave::WriteAddressed:
      slave_.read(&rx_data, 1);
      break;
    default:
      // No data received
      ThisThread::sleep_for(1);
      break;
    }
  }
}

/**
 *
 * 0x00 spec_co: Gas Concentration: 4
 * 0x01 spec_co: Temperature:       2
 * 0x02 spec_co: Relative Humidity  2
 *
 * 0x10 sgp30: Carbon Dioxide       2
 * 0x11 sgp30: VOC                  2
 *
 * 0x20 i HTS221 Temperature        4
 * 0x21 i HTS221 Humidity           4
 * 0x22 i LPS22HB Temperature       4
 * 0x23 i LPS22HB Pressure          4
 * 0x24 i Time of Flight            4
 * 0x25 i Accelerometer             12
 * 0x26 i Gyroscope                 12
 * 0x27 i Magnetometer              12
 *
 * 0x30 hb Heartbeat Data           8
 *
 * 0x40 gps GPS Data                8
 */
void SlaveCommunication::transmit_sensor_data(uint8_t address) {
  switch (address) {
  // SPEC CO SENSOR
  case 0x00:
    send_spec_co_gas_concentration();
    break;
  case 0x01:
    send_spec_co_temperature();
    break;
  case 0x02:
    send_spec_co_humidity();
    break;
  // INTERNAL SENSORS
  case 0x20:
    send_hts221_temperature();
    break;
  case 0x21:
    send_hts221_humidity();
    break;
  case 0x22:
    send_lps22hb_temperature();
    break;
  case 0x23:
    send_lps22hb_pressure();
    break;
  case 0x24:
    send_time_of_flight();
    break;
  case 0x25:
    send_accelerometer();
    break;
  case 0x26:
    send_gyroscope();
    break;
  case 0x27:
    send_magnetometer();
    break;
  // HEARTBEAT SENSOR
  case 0x30:
    send_heartbeat_data();
    break;
  // GPS
  case 0x40:
    send_gps_coordinates_data();
    break;
  default:
    break;
  }
}

void SlaveCommunication::send_spec_co_gas_concentration(void) {
  uint8_t tx_data[ARR_SIZE] = {0};
  uint32_to_uint8_array(co_.get_gas_concentration(), tx_data);
  slave_.write((const char *)tx_data, ARR_SIZE);
}

void SlaveCommunication::send_spec_co_temperature() {
  uint8_t tx_data[ARR_SIZE] = {0};
  uint32_to_uint8_array(static_cast<uint32_t>(co_.get_temperature()), tx_data);
  slave_.write((const char *)tx_data, ARR_SIZE >> 1);
}

void SlaveCommunication::send_spec_co_humidity() {
  uint8_t tx_data[ARR_SIZE] = {0};
  uint32_to_uint8_array(static_cast<uint32_t>(co_.get_relative_humidity()),
                        tx_data);
  slave_.write((const char *)tx_data, ARR_SIZE >> 1);
}

void SlaveCommunication::send_hts221_temperature() {
  uint8_t tx_data[ARR_SIZE] = {0};
  const internal_sensor::data_s &data = internal_sensor::get_sensor_data();
  const uint32_t rdata =
      convert_float_to_uint32_float_structure(data.hts221_temperature);
  uint32_to_uint8_array(rdata, tx_data);
  slave_.write((const char *)tx_data, ARR_SIZE);
}

void SlaveCommunication::send_hts221_humidity() {
  uint8_t tx_data[ARR_SIZE] = {0};
  const internal_sensor::data_s &data = internal_sensor::get_sensor_data();
  const uint32_t rdata =
      convert_float_to_uint32_float_structure(data.hts221_humidity);
  uint32_to_uint8_array(rdata, tx_data);
  slave_.write((const char *)tx_data, ARR_SIZE);
}

void SlaveCommunication::send_lps22hb_temperature() {
  uint8_t tx_data[ARR_SIZE] = {0};
  const internal_sensor::data_s &data = internal_sensor::get_sensor_data();
  const uint32_t rdata =
      convert_float_to_uint32_float_structure(data.lps22hb_temperature);
  uint32_to_uint8_array(rdata, tx_data);
  slave_.write((const char *)tx_data, ARR_SIZE);
}

void SlaveCommunication::send_lps22hb_pressure() {
  uint8_t tx_data[ARR_SIZE] = {0};
  const internal_sensor::data_s &data = internal_sensor::get_sensor_data();
  const uint32_t rdata =
      convert_float_to_uint32_float_structure(data.lps22hb_pressure);
  uint32_to_uint8_array(rdata, tx_data);
  slave_.write((const char *)tx_data, ARR_SIZE);
}

void SlaveCommunication::send_time_of_flight() {
  uint8_t tx_data[ARR_SIZE] = {0};
  const internal_sensor::data_s &data = internal_sensor::get_sensor_data();
  uint32_to_uint8_array(data.distance, tx_data);
  slave_.write((const char *)tx_data, ARR_SIZE);
}

void SlaveCommunication::send_accelerometer(void) {
  uint8_t tx_data[12] = {0};
  const internal_sensor::data_s &data = internal_sensor::get_sensor_data();
  uint32_to_uint8_array(static_cast<uint32_t>(data.acceleration_axes[0]),
                        tx_data);
  uint32_to_uint8_array(static_cast<uint32_t>(data.acceleration_axes[1]),
                        tx_data + 4);
  uint32_to_uint8_array(static_cast<uint32_t>(data.acceleration_axes[2]),
                        tx_data + 8);
  slave_.write((const char *)tx_data, ARR_SIZE * 3);
}

void SlaveCommunication::send_gyroscope() {
  uint8_t tx_data[12] = {0};
  const internal_sensor::data_s &data = internal_sensor::get_sensor_data();
  uint32_to_uint8_array(static_cast<uint32_t>(data.gyroscope_axes[0]), tx_data);
  uint32_to_uint8_array(static_cast<uint32_t>(data.gyroscope_axes[1]),
                        tx_data + 4);
  uint32_to_uint8_array(static_cast<uint32_t>(data.gyroscope_axes[2]),
                        tx_data + 8);
  slave_.write((const char *)tx_data, ARR_SIZE * 3);
}

void SlaveCommunication::send_magnetometer() {
  uint8_t tx_data[12] = {0};
  const internal_sensor::data_s &data = internal_sensor::get_sensor_data();
  uint32_to_uint8_array(static_cast<uint32_t>(data.magnetometer_axes[0]),
                        tx_data);
  uint32_to_uint8_array(static_cast<uint32_t>(data.magnetometer_axes[1]),
                        tx_data + 4);
  uint32_to_uint8_array(static_cast<uint32_t>(data.magnetometer_axes[2]),
                        tx_data + 8);
  slave_.write((const char *)tx_data, ARR_SIZE * 3);
}

void SlaveCommunication::send_heartbeat_data() {
  uint8_t tx_data[8] = {0};
  const heartbeat::bioData &data = hb_.get_hb_data();
  uint16_to_uint8_array(static_cast<uint16_t>(data.heartRate), tx_data);
  uint16_to_uint8_array(static_cast<uint16_t>(0xFF00 | data.confidence),
                        tx_data + 2);
  uint16_to_uint8_array(static_cast<uint16_t>(data.oxygen), tx_data + 4);
  uint16_to_uint8_array(static_cast<uint16_t>(0xFF00 | data.status),
                        tx_data + 6);

  slave_.write((const char *)tx_data, ARR_SIZE * 2);
}

void SlaveCommunication::send_gps_coordinates_data() {
  uint8_t tx_data[ARR_SIZE * 2] = {0};
  const gps::gps_coordinates_s &data = gps_.get_gps_coordinates();

  // Hardcoded values (Only for testing I2C data transmission to TI CC1352_R1)
  // data.latitude = -37.123456;
  // data.longitude = 121.987650;

  debugPrintf("I2C_GPS Lat: %lu | Long: %lu\r\n", (uint32_t)(data.latitude),
              (uint32_t)(data.longitude));
  uint32_t lat_data = convert_float_to_uint32_float_structure(data.latitude);
  uint32_t long_data = convert_float_to_uint32_float_structure(data.longitude);

  uint32_to_uint8_array(static_cast<uint32_t>(lat_data), tx_data);
  uint32_to_uint8_array(static_cast<uint32_t>(long_data), tx_data + 4);
  slave_.write((const char *)tx_data, ARR_SIZE * 2);
}
} // namespace i2c_slave
