#ifndef USER_HEARTBEATLOGIC_H_
#define USER_HEARTBEATLOGIC_H_

#include "mbed.h"
#include <string.h>
#include <stdint.h>

#define MODE_ONE 0x01
#define MODE_TWO 0x02
#define MAXFAST_ARRAY_SIZE 6
#define MAXFAST_EXTENDED_DATA 5

static const uint32_t DELAY_AFTER_HEARTBEAT_INITIALIZE_MILLISECONDS = 4000;
static const uint32_t HEARTBEAT_TASK_DELAY_MILLISECONDS = 250;

namespace hb_sensor
{
  struct bioData
  {
    uint32_t irLed;
    uint32_t redLed;
    uint16_t heartRate;  // LSB = 0.1bpm
    uint8_t confidence;  // 0-100% LSB = 1%
    uint16_t oxygen;     // 0-100% LSB = 1%
    uint8_t status;      // 0: Success, 1: Not Ready, 2: Object Detectected, 3: Finger
                         // Detected
    float rValue;        // -- Algorithm Mode 2 vv
    int8_t extStatus;    // --
    uint8_t reserveOne;  // --
    uint8_t resserveTwo; // -- Algorithm Mode 2 ^^
  };

  struct version
  {
    uint8_t major;
    uint8_t minor;
    uint8_t revision;
  };

  // Status Bytes are communicated back after every I2C transmission and
  // are indicators of success or failure of the previous transmission.
  enum READ_STATUS_BYTE_VALUE
  {
    SUCCESS = 0x00,
    ERR_UNAVAIL_CMD,
    ERR_UNAVAIL_FUNC,
    ERR_DATA_FORMAT,
    ERR_INPUT_VALUE,
    ERR_TRY_AGAIN,
    ERR_BTLDR_GENERAL = 0x80,
    ERR_BTLDR_CHECKSUM,
    ERR_BTLDR_AUTH,
    ERR_BTLDR_INVALID_APP,
    ERR_UNKNOWN = 0xFF
  };

  // The family register bytes are the larger umbrella for all the Index and
  // Write Bytes listed below. You can not reference a nestled byte without first
  // referencing it's larger category: Family Register Byte.
  enum FAMILY_REGISTER_BYTES
  {
    HUB_STATUS = 0x00,
    SET_DEVICE_MODE,
    READ_DEVICE_MODE,
    OUTPUT_MODE = 0x10,
    READ_OUTPUT_MODE,
    READ_DATA_OUTPUT,
    READ_DATA_INPUT,
    WRITE_INPUT,
    WRITE_REGISTER = 0x40,
    READ_REGISTER,
    READ_ATTRIBUTES_AFE,
    DUMP_REGISTERS,
    ENABLE_SENSOR,
    READ_SENSOR_MODE,
    CHANGE_ALGORITHM_CONFIG = 0x50,
    READ_ALGORITHM_CONFIG,
    ENABLE_ALGORITHM,
    BOOTLOADER_FLASH = 0x80,
    BOOTLOADER_INFO,
    IDENTITY = 0xFF
  };

  // Write Bytes under Family Byte: OUTPUT_MODE (0x10) and Index byte: SET_FORMAT
  // (0x00)
  enum OUTPUT_MODE_WRITE_BYTE
  {
    PAUSE = 0x00,
    SENSOR_DATA,
    ALGO_DATA,
    SENSOR_AND_ALGORITHM,
    PAUSE_TWO,
    SENSOR_COUNTER_BYTE,
    ALGO_COUNTER_BYTE,
    SENSOR_ALGO_COUNTER
  };

  // Index Byte under the Family Byte: READ_DATA_OUTPUT (0x12)
  enum FIFO_OUTPUT_INDEX_BYTE
  {
    NUM_SAMPLES,
    READ_DATA
  };

  // Index Byte under the Family Byte: ENABLE_SENSOR (0x44)
  enum SENSOR_ENABLE_INDEX_BYTE
  {
    ENABLE_MAX30101 = 0x03,
    ENABLE_ACCELEROMETER
  };

  // Index Byte under the Family Byte: CHANGE_ALGORITHM_CONFIG (0x50)
  enum ALGORITHM_CONFIG_INDEX_BYTE
  {
    SET_TARG_PERC = 0x00,
    SET_STEP_SIZE = 0x00,
    SET_SENSITIVITY = 0x00,
    SET_AVG_SAMPLES = 0x00,
    SET_PULSE_OX_COEF = 0x02,
  };

  // Index Bytes under the Family Byte: READ_ALGORITHM_CONFIG (0x51)
  enum READ_ALGORITHM_INDEX_BYTE
  {
    READ_AGC_PERCENTAGE = 0x00,
    READ_AGC_STEP_SIZE = 0x00,
    READ_AGC_SENSITIVITY = 0x00,
    READ_AGC_NUM_SAMPLES = 0x00,
    READ_MAX_FAST_COEF = 0x02
  };

  // Write Bytes under the Family Byte: READ_ALGORITHM_CONFIG (0x51) and Index
  // Byte: READ_ALGORITHM_INDEX_BYTE - AGC
  enum READ_AGC_ALGO_WRITE_BYTE
  {
    READ_AGC_PERC_ID = 0x00,
    READ_AGC_STEP_SIZE_ID,
    READ_AGC_SENSITIVITY_ID,
    READ_AGC_NUM_SAMPLES_ID,
    READ_MAX_FAST_COEF_ID = 0x0B
  };

  // Index Byte under the Family Byte: ENABLE_ALGORITHM (0x52).
  enum ALGORITHM_MODE_ENABLE_INDEX_BYTE
  {
    ENABLE_AGC_ALGO = 0x00,
    ENABLE_WHRM_ALGO = 0x02
  };

  class hb_sensor_class
  {
  public:
    hb_sensor_class(PinName sda, PinName scl) : i2c_hb(sda, scl)
    {
    }

    uint8_t begin(void);

    uint8_t configBpm(uint8_t);

    struct bioData readBpm(void);

    uint8_t setOutputMode(uint8_t);

    uint8_t setFifoThreshold(uint8_t);

    uint8_t agcAlgoControl(uint8_t);

    uint8_t max30101Control(uint8_t);

    uint8_t maximFastAlgoControl(uint8_t);

    uint8_t readAlgoSamples(void);

    uint8_t readSensorHubStatus(void);

    uint8_t numSamplesOutFifo(void);

    void heartbeat__initialize_application_mode();

    uint8_t enableWrite(uint8_t, uint8_t, uint8_t);

    uint8_t writeByte_1(uint8_t, uint8_t, uint8_t);

    uint8_t readByte_1(uint8_t, uint8_t);

    uint8_t readByte_2(uint8_t, uint8_t, uint8_t);

    uint8_t readFillArray(uint8_t, uint8_t, uint8_t, uint8_t *);

    void readSensorVersion();

  private:
    I2C i2c_hb;

    uint8_t _address;

    uint8_t _userSelectedMode;

    uint8_t _sampleRate;

    uint8_t bpmArr[MAXFAST_ARRAY_SIZE];

    uint8_t bpmArrTwo[MAXFAST_ARRAY_SIZE + MAXFAST_EXTENDED_DATA];
    const uint8_t BIO_ADDRESS = 0xAA;
    const uint8_t INCORR_PARAM = 0xEE;
    const uint32_t ENABLE_CMD_DELAY_MS = 45; // Milliseconds
    const uint32_t CMD_DELAY_MS = 6;         // Milliseconds
    const uint8_t SET_FORMAT = 0x00;
    const uint8_t ENABLE = 0x01;
    const uint8_t WRITE_SET_THRESHOLD = 0x01;
  };

}
#endif