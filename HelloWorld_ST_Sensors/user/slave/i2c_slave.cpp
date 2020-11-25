#include "i2c_slave.hpp"

#include "internal_sensors.hpp"

static void uint32_to_uint8_array(uint32_t data, uint8_t *arr) {
  arr[0] = (data >> (0 * 8)) & 0xFF;
  arr[1] = (data >> (1 * 8)) & 0xFF;
  arr[2] = (data >> (2 * 8)) & 0xFF;
  arr[3] = (data >> (3 * 8)) & 0xFF;
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
      printf("Write General\r\n");
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
 */
void SlaveCommunication::transmit_sensor_data(uint8_t address) {
  switch (address) {
  // SPEC CO
  case 0x00:
    send_spec_co_gas_concentration();
    break;
  case 0x01:
    send_spec_co_temperature();
    break;
  case 0x02:
    send_spec_co_humidity();
    break;

  // SGP30
  case 0x10:
    send_sgp30_co2();
    break;
  case 0x11:
    send_sgp30_voc();
    break;

  // INTERNAL
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

void SlaveCommunication::send_sgp30_co2() {
  uint8_t tx_data[ARR_SIZE] = {0};
  uint32_to_uint8_array(static_cast<uint32_t>(sgp30_.get_co2()), tx_data);
  slave_.write((const char *)tx_data, ARR_SIZE >> 1);
}

void SlaveCommunication::send_sgp30_voc() {
  uint8_t tx_data[ARR_SIZE] = {0};
  uint32_to_uint8_array(static_cast<uint32_t>(sgp30_.get_voc()), tx_data);
  slave_.write((const char *)tx_data, ARR_SIZE >> 1);
}

// TODO, Convert to float
void SlaveCommunication::send_hts221_temperature() {
  uint8_t tx_data[ARR_SIZE] = {0};
  const internal_sensor::data_s &data = internal_sensor::get_sensor_data();
  uint32_to_uint8_array(static_cast<uint32_t>(data.hts221_temperature),
                        tx_data);
  slave_.write((const char *)tx_data, ARR_SIZE);
}

// TODO, Convert to float
void SlaveCommunication::send_hts221_humidity() {
  uint8_t tx_data[ARR_SIZE] = {0};
  const internal_sensor::data_s &data = internal_sensor::get_sensor_data();
  uint32_to_uint8_array(static_cast<uint32_t>(data.hts221_humidity), tx_data);
  slave_.write((const char *)tx_data, ARR_SIZE);
}

// TODO, Convert to float
void SlaveCommunication::send_lps22hb_temperature() {
  uint8_t tx_data[ARR_SIZE] = {0};
  const internal_sensor::data_s &data = internal_sensor::get_sensor_data();
  uint32_to_uint8_array(static_cast<uint32_t>(data.lps22hb_temperature),
                        tx_data);
  slave_.write((const char *)tx_data, ARR_SIZE);
}

// TODO, Convert to float
void SlaveCommunication::send_lps22hb_pressure() {
  uint8_t tx_data[ARR_SIZE] = {0};
  const internal_sensor::data_s &data = internal_sensor::get_sensor_data();
  uint32_to_uint8_array(static_cast<uint32_t>(data.lps22hb_pressure), tx_data);
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

} // namespace i2c_slave
