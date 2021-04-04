#include "user/heartbeat/heartbeat.hpp"

#define DEBUG_PRINTF 0

#if DEBUG_PRINTF
#define debugPrintf(...) debugPrintf(__VA_ARGS__)
#else
#define debugPrintf(...)
#endif

namespace heartbeat {

static struct bioData body;

uint8_t sparkfun_MAX32664::begin(void) {
  heartbeat__initialize_application_mode();
  uint8_t responseByte = readByte_1(READ_DEVICE_MODE, 0x00);
  return responseByte;
}

uint8_t sparkfun_MAX32664::configBpm(uint8_t mode) {
  uint8_t statusChauf = 0;

  if (mode == MODE_ONE || mode == MODE_TWO) {
  } else {
    return INCORR_PARAM;
  }

  statusChauf = setOutputMode(ALGO_DATA); // Just the data
  if (statusChauf != SUCCESS) {
    return statusChauf;
  }

  statusChauf = setFifoThreshold(0x01); // One sample before interrupt is fired.
  if (statusChauf != SUCCESS) {
    return statusChauf;
  }

  statusChauf = agcAlgoControl(ENABLE); // One sample before interrupt is fired.
  if (statusChauf != SUCCESS) {
    return statusChauf;
  }

  statusChauf = max30101Control(ENABLE);
  if (statusChauf != SUCCESS) {
    return statusChauf;
  }

  statusChauf = maximFastAlgoControl(mode);
  if (statusChauf != SUCCESS) {
    return statusChauf;
  }

  _userSelectedMode = mode;
  _sampleRate = readAlgoSamples();

  ThisThread::sleep_for(1000);
  return SUCCESS;
}

// This function takes the 8 bytes from the FIFO buffer related to the wrist
// heart rate algortihm: heart rate (uint16_t), confidence (uint8_t) , SpO2
// (uint16_t), and the finger detected status (uint8_t). Note that the the
// algorithm is stated as "wrist" though the sensor only works with the finger.
// The data is loaded into the whrmFifo and returned.
struct bioData sparkfun_MAX32664::readBpm(void) {
  struct bioData libBpm;
  uint8_t statusChauf; // The status chauffeur captures return values.

  statusChauf = readSensorHubStatus();

  if (statusChauf == 1) { // Communication Error
    libBpm.heartRate = 0;
    libBpm.confidence = 0;
    libBpm.oxygen = 0;
    return libBpm;
  }

  numSamplesOutFifo();

  if (_userSelectedMode == MODE_ONE) {
    readFillArray(READ_DATA_OUTPUT, READ_DATA, MAXFAST_ARRAY_SIZE, bpmArr);

    // Heart Rate formatting
    libBpm.heartRate = (uint16_t)((bpmArr[0]) << 8);
    libBpm.heartRate |= (bpmArr[1]);
    libBpm.heartRate /= 10;

    // Confidence formatting
    libBpm.confidence = bpmArr[2];

    // Blood oxygen level formatting
    libBpm.oxygen = (uint16_t)(bpmArr[3] << 8);
    libBpm.oxygen |= bpmArr[4];
    libBpm.oxygen /= 10;

    //"Machine State" - has a finger been detected?
    libBpm.status = bpmArr[5];

    return libBpm;
  }

  else if (_userSelectedMode == MODE_TWO) {
    readFillArray(READ_DATA_OUTPUT, READ_DATA,
                  MAXFAST_ARRAY_SIZE + MAXFAST_EXTENDED_DATA, bpmArrTwo);

    // Heart Rate formatting
    libBpm.heartRate = (uint16_t)((bpmArrTwo[0]) << 8);
    libBpm.heartRate |= (bpmArrTwo[1]);
    libBpm.heartRate /= 10;

    // Confidence formatting
    libBpm.confidence = bpmArrTwo[2];

    // Blood oxygen level formatting
    libBpm.oxygen = (uint16_t)((bpmArrTwo[3]) << 8);
    libBpm.oxygen |= bpmArrTwo[4];
    libBpm.oxygen /= 10.0;

    //"Machine State" - has a finger been detected?
    libBpm.status = bpmArrTwo[5];

    // Sp02 r Value formatting
    uint16_t tempVal = (uint16_t)((bpmArrTwo[6]) << 8);
    tempVal |= bpmArrTwo[7];
    libBpm.rValue = tempVal;
    libBpm.rValue /= 10.0;

    // Extended Machine State formatting
    libBpm.extStatus = bpmArrTwo[8];

    // There are two additional bytes of data that were requested but that
    // have not been implemented in firmware 10.1 so will not be saved to
    // user's data.
    return libBpm;
  }

  else {
    libBpm.heartRate = 0;
    libBpm.confidence = 0;
    libBpm.oxygen = 0;
    return libBpm;
  }
}

// Heartbeat Mid-level APIs

// Family Byte: OUTPUT_MODE (0x10), Index Byte: SET_FORMAT (0x00),
// Write Byte : outputType (Parameter values in OUTPUT_MODE_WRITE_BYTE)
uint8_t sparkfun_MAX32664::setOutputMode(uint8_t outputType) {
  if (outputType > SENSOR_ALGO_COUNTER) // Bytes between 0x00 and 0x07
    return INCORR_PARAM;

  // Check that communication was successful, not that the IC is outputting
  // correct format.
  uint8_t statusByte = writeByte_1(OUTPUT_MODE, SET_FORMAT, outputType);
  if (statusByte != SUCCESS)
    return statusByte;
  else
    return SUCCESS;
}

// Family Byte: OUTPUT_MODE(0x10), Index Byte: WRITE_SET_THRESHOLD (0x01), Write
// byte: intThres (parameter - value betwen 0 and 0xFF). This function changes
// the threshold for the FIFO interrupt bit/pin. The interrupt pin is the MFIO
// pin which is set to INPUT after IC initialization (begin).
uint8_t sparkfun_MAX32664::setFifoThreshold(uint8_t intThresh) {
  // Checks that there was succesful communcation, not that the threshold was
  // set correctly.
  uint8_t statusByte = writeByte_1(OUTPUT_MODE, WRITE_SET_THRESHOLD, intThresh);
  if (statusByte != SUCCESS)
    return statusByte;
  else
    return SUCCESS;
}

// Family Byte: ENABLE_ALGORITHM (0x52), Index Byte:
// ENABLE_AGC_ALGO (0x00)
// This function enables (one) or disables (zero) the automatic gain control
// algorithm.
uint8_t sparkfun_MAX32664::agcAlgoControl(uint8_t enable) {
  if (enable == 0 || enable == 1) {
  } else
    return INCORR_PARAM;

  uint8_t statusByte = enableWrite(ENABLE_ALGORITHM, ENABLE_AGC_ALGO, enable);
  if (statusByte != SUCCESS)
    return statusByte;
  else
    return SUCCESS;
}

// Family Byte: ENABLE_SENSOR (0x44), Index Byte: ENABLE_MAX30101 (0x03), Write
// Byte: senSwitch  (parameter - 0x00 or 0x01).
// This function enables the MAX30101.
uint8_t sparkfun_MAX32664::max30101Control(uint8_t senSwitch) {
  if (senSwitch == 0 || senSwitch == 1) {
  } else
    return INCORR_PARAM;

  // Check that communication was successful, not that the sensor is enabled.
  uint8_t statusByte = enableWrite(ENABLE_SENSOR, ENABLE_MAX30101, senSwitch);
  if (statusByte != SUCCESS)
    return statusByte;
  else
    return SUCCESS;
}

// Family Byte: ENABLE_ALGORITHM (0x52), Index Byte:
// ENABLE_WHRM_ALGO (0x02)
// This function enables (one) or disables (zero) the wrist heart rate monitor
// algorithm.
uint8_t sparkfun_MAX32664::maximFastAlgoControl(uint8_t mode) {
  if (mode == 0 || mode == 1 || mode == 2) {
  } else
    return INCORR_PARAM;

  uint8_t statusByte = enableWrite(ENABLE_ALGORITHM, ENABLE_WHRM_ALGO, mode);
  if (statusByte != SUCCESS)
    return statusByte;
  else
    return SUCCESS;
}

// Family Byte: READ_ALGORITHM_CONFIG (0x51), Index Byte:
// READ_AGC_NUM_SAMPLES (0x00), Write Byte: READ_AGC_NUM_SAMPLES_ID (0x03)
// This function changes the number of samples that are averaged.
// It takes a paramater of zero to 255.
uint8_t sparkfun_MAX32664::readAlgoSamples(void) {
  uint8_t samples = readByte_2(READ_ALGORITHM_CONFIG, READ_AGC_NUM_SAMPLES,
                               READ_AGC_NUM_SAMPLES_ID);
  return samples;
}

// Family Byte: HUB_STATUS (0x00), Index Byte: 0x00, No Write Byte.
// The following function checks the status of the FIFO.
uint8_t sparkfun_MAX32664::readSensorHubStatus(void) {
  uint8_t status = readByte_1(0x00, 0x00); // Just family and index byte.
  return status;                           // Will return 0x00
}

// Family Byte: READ_DATA_OUTPUT (0x12), Index Byte: NUM_SAMPLES (0x00), Write
// Byte: NONE
// This function returns the number of samples available in the FIFO.
uint8_t sparkfun_MAX32664::numSamplesOutFifo(void) {
  uint8_t sampAvail = readByte_1(READ_DATA_OUTPUT, NUM_SAMPLES);
  return sampAvail;
}

// Utilities (Heartbeat Low-level APIs)

void sparkfun_MAX32664::heartbeat__initialize_application_mode(void) {
  DigitalOut mfio_pin(PD_14);
  DigitalOut rst_pin(PB_0);

  mfio_pin.write(1);
  rst_pin.write(0);

  ThisThread::sleep_for(10);

  rst_pin.write(1);

  ThisThread::sleep_for(1000);
}

uint8_t sparkfun_MAX32664::enableWrite(uint8_t _familyByte, uint8_t _indexByte,
                                       uint8_t _enableByte) {
  uint8_t statusByte;
  const uint8_t number_of_bytes_to_read = 1;
  const uint8_t number_of_bytes_to_write = 3;

  char readBuffer[number_of_bytes_to_read] = {0};
  char writeBuffer[number_of_bytes_to_write] = {0};

  writeBuffer[0] = static_cast<char>(_familyByte);
  writeBuffer[1] = static_cast<char>(_indexByte);
  writeBuffer[2] = static_cast<char>(_enableByte);
  i2c_hb.write(BIO_ADDRESS, writeBuffer, number_of_bytes_to_write);

  ThisThread::sleep_for(ENABLE_CMD_DELAY_MS);

  i2c_hb.read(BIO_ADDRESS, readBuffer, number_of_bytes_to_read);

  statusByte = static_cast<uint8_t>(readBuffer[0]);

  return statusByte;
}

uint8_t sparkfun_MAX32664::writeByte_1(uint8_t _familyByte, uint8_t _indexByte,
                                       uint8_t _writeByte) {
  uint8_t statusByte;
  const uint8_t number_of_bytes_to_read = 1;
  const uint8_t number_of_bytes_to_write = 3;
  char readBuffer[number_of_bytes_to_read] = {0};
  char writeBuffer[number_of_bytes_to_write] = {0};

  writeBuffer[0] = static_cast<char>(_familyByte);
  writeBuffer[1] = static_cast<char>(_indexByte);
  writeBuffer[2] = static_cast<char>(_writeByte);

  i2c_hb.write(BIO_ADDRESS, writeBuffer, number_of_bytes_to_write);

  ThisThread::sleep_for(CMD_DELAY_MS);

  i2c_hb.read(BIO_ADDRESS, readBuffer, number_of_bytes_to_read);

  statusByte = static_cast<uint8_t>(readBuffer[0]);

  return statusByte;
}

uint8_t sparkfun_MAX32664::readByte_1(uint8_t _familyByte, uint8_t _indexByte) {
  uint8_t statusByte;
  uint8_t returnByte;
  const uint8_t number_of_bytes_to_read = 2;
  const uint8_t number_of_bytes_to_write = 2;
  char readBuffer[number_of_bytes_to_read] = {0};
  char writeBuffer[number_of_bytes_to_write] = {0};

  writeBuffer[0] = static_cast<char>(_familyByte);
  writeBuffer[1] = static_cast<char>(_indexByte);

  i2c_hb.write(BIO_ADDRESS, writeBuffer, number_of_bytes_to_write);

  ThisThread::sleep_for(CMD_DELAY_MS);

  i2c_hb.read(BIO_ADDRESS, readBuffer, number_of_bytes_to_read);

  statusByte = static_cast<uint8_t>(readBuffer[0]);
  returnByte = static_cast<uint8_t>(readBuffer[1]);

  if (statusByte) {
    return statusByte;
  }

  return returnByte;
}

uint8_t sparkfun_MAX32664::readByte_2(uint8_t _familyByte, uint8_t _indexByte,
                                      uint8_t _writeByte) {
  uint8_t statusByte;
  uint8_t returnByte;
  const uint8_t number_of_bytes_to_read = 2;
  const uint8_t number_of_bytes_to_write = 3;
  char readBuffer[number_of_bytes_to_read] = {0};
  char writeBuffer[number_of_bytes_to_write] = {0};

  writeBuffer[0] = static_cast<char>(_familyByte);
  writeBuffer[1] = static_cast<char>(_indexByte);
  writeBuffer[2] = static_cast<char>(_writeByte);

  i2c_hb.write(BIO_ADDRESS, writeBuffer, number_of_bytes_to_write);

  ThisThread::sleep_for(CMD_DELAY_MS);

  i2c_hb.read(BIO_ADDRESS, readBuffer, number_of_bytes_to_read);

  statusByte = static_cast<uint8_t>(readBuffer[0]);
  returnByte = static_cast<uint8_t>(readBuffer[1]);
  if (statusByte) {
    return statusByte;
  }

  return returnByte;
}

uint8_t sparkfun_MAX32664::readFillArray(uint8_t _familyByte,
                                         uint8_t _indexByte, uint8_t arraySize,
                                         uint8_t *array) {
  uint8_t statusByte;

  uint8_t number_of_bytes_to_read = (1 + arraySize);
  const uint8_t number_of_bytes_to_write = 2;

  char readBuffer[number_of_bytes_to_read] = {0};
  char writeBuffer[number_of_bytes_to_write] = {0};

  writeBuffer[0] = static_cast<char>(_familyByte);
  writeBuffer[1] = static_cast<char>(_indexByte);

  i2c_hb.write(BIO_ADDRESS, writeBuffer, number_of_bytes_to_write);

  ThisThread::sleep_for(CMD_DELAY_MS);

  i2c_hb.read(BIO_ADDRESS, readBuffer, number_of_bytes_to_read);

  statusByte = static_cast<uint8_t>(readBuffer[0]);

  if (statusByte) // Error scenario when statusByte is not equal to 0
  {
    for (int i = 0; i < arraySize; i++) {
      array[i] = 0;
    }
  } else {
    for (int i = 1; i <= arraySize; i++) {
      array[i - 1] = static_cast<uint8_t>(readBuffer[i]);
    }
  }
  return statusByte;
}

void sparkfun_MAX32664::hb_start(void) {
  uint8_t response_code;
  response_code = begin();
  if (!response_code) {
    debugPrintf("Heartbeat sensor started\r\n");
    response_code = configBpm(0x01);
    if (!response_code) {
      debugPrintf("Heartbeat sensor successfully configured\r\n");
      ThisThread::sleep_for(DELAY_AFTER_HEARTBEAT_INITIALIZE_MILLISECONDS);
      thread_.start(callback(this, &sparkfun_MAX32664::hb_thread_task));
    } else {
      debugPrintf(
          "Could not configure the Heartbeat sensor. response_code = %d\r\n",
          response_code);
    }
  } else {
    debugPrintf("Could not communicate with the sensor\r\n");
  }
}

void sparkfun_MAX32664::hb_thread_task(void) {
  while (1) {
    body = readBpm();

    ThisThread::sleep_for(HEARTBEAT_TASK_DELAY_MILLISECONDS); // 250 millseconds
  }
}

const struct bioData &get_hb_data(void) { return body; }

} // namespace heartbeat
