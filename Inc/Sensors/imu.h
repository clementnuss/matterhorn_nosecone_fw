/*
 * imu.h
 *
 *  Created on: 2 Apr 2018
 *      Author: Cl�ment Nussbaumer
 */

#ifndef IMU_IMU_H_
#define IMU_IMU_H_

#include "cmsis_os.h"
#include "Misc/Common.h"

HAL_StatusTypeDef initIMU ();
HAL_StatusTypeDef waitTssState (uint32_t timeoutmMs, uint8_t tss_spi_state);
HAL_StatusTypeDef sendCommand (uint8_t command, uint8_t* args, uint8_t argsSize,
                               uint8_t* rxData, uint16_t rxSize);

HAL_StatusTypeDef getIMUdataDMA ();
void processBuffer (IMU_data* tss_data);
void increasePosUntilTssReady (uint16_t*pos);

void insertCommandIntoBuffer (uint8_t* commandBuffer, uint16_t* pos,
                              uint8_t tss_command,
                              uint16_t tss_send_packet_number);


void TK_IMU (void const * argument);

typedef enum TSS_Stream_Command_Enum
{
  TSS_GET_TARED_ORIENTATION_AS_QUATERNION = 0x00,
  TSS_GET_TARED_ORIENTATION_AS_EULER_ANGLES = 0x01,
  TSS_GET_TARED_ORIENTATION_AS_ROTATION_MATRIX = 0x02,
  TSS_GET_TARED_ORIENTATION_AS_AXIS_ANGLE = 0x03,
  TSS_GET_TARED_ORIENTATION_AS_TWO_VECTOR = 0x04,
  TSS_GET_DIFFERENCE_QUATERNION = 0x05,
  TSS_GET_UNTARED_ORIENTATION_AS_QUATERNION = 0x06,
  TSS_GET_UNTARED_ORIENTATION_AS_EULER_ANGLES = 0x07,
  TSS_GET_UNTARED_ORIENTATION_AS_ROTATION_MATRIX = 0x08,
  TSS_GET_UNTARED_ORIENTATION_AS_AXIS_ANGLE = 0x09,
  TSS_GET_UNTARED_ORIENTATION_AS_TWO_VECTOR = 0x0a,
  TSS_GET_TARED_TWO_VECTOR_IN_SENSOR_FRAME = 0x0b,
  TSS_GET_UNTARED_TWO_VECTOR_IN_SENSOR_FRAME = 0x0c,
  TSS_GET_ALL_NORMALIZED_COMPONENT_SENSOR_DATA = 0x20,
  TSS_GET_NORMALIZED_GYRO_RATE = 0x21,
  TSS_GET_NORMALIZED_ACCELEROMETER_VECTOR = 0x22,
  TSS_GET_NORMALIZED_COMPASS_VECTOR = 0x23,
  TSS_GET_ALL_CORRECTED_COMPONENT_SENSOR_DATA = 0x25,
  TSS_GET_CORRECTED_GYRO_RATE = 0x26,
  TSS_GET_CORRECTED_ACCELEROMETER_VECTOR = 0x27,
  TSS_GET_CORRECTED_COMPASS_VECTOR = 0x28,
  TSS_GET_CORRECTED_LINEAR_ACCELERATION_IN_GLOBAL_SPACE = 0x29,
  TSS_GET_TEMPERATURE_C = 0x2b,
  TSS_GET_TEMPERATURE_F = 0x2c,
  TSS_GET_CONFIDENCE_FACTOR = 0x2d,
  TSS_GET_ALL_RAW_COMPONENT_SENSOR_DATA = 0x40,
  TSS_GET_RAW_GYROSCOPE_RATE = 0x41,
  TSS_GET_RAW_ACCELEROMETER_DATA = 0x42,
  TSS_GET_RAW_COMPASS_DATA = 0x43,
  TSS_GET_BATTERY_VOLTAGE = 0xc9,
  TSS_GET_BATTERY_PERCENT_REMAINING = 0xca,
  TSS_GET_BATTERY_STATUS = 0xcb,
  TSS_GET_BUTTON_STATE = 0xfa,
  TSS_NULL = 0xff
} TSS_Stream_Command_Enum;

typedef enum TSS_Command_Enum
{
  TSS_SET_EULER_ANGLE_DECOMPOSITION_ORDER = 0x10,
  TSS_SET_MAGNETORESISTIVE_THRESHOLD = 0x11,
  TSS_SET_ACCELEROMETER_RESISTANCE_THRESHOLD = 0x12,
  TSS_OFFSET_WITH_CURRENT_ORIENTATION = 0x13,
  TSS_RESET_BASE_OFFSET = 0x14,
  TSS_OFFSET_WITH_QUATERNION = 0x15,
  TSS_SET_BASE_OFFSET_WITH_CURRENT_ORIENTATION = 0x16,
  TSS_SET_CHECK_LONG_COMMANDS = 0x19,
  TSS_SET_PIN_MODE = 0x1d,
  TSS_GET_PIN_MODE = 0x1e,
  TSS_GET_INTERRUPT_STATUS = 0x1f,
  TSS_TURN_ON_MASSS_TORAGE = 0x39,
  TSS_TURN_OFF_MASS_STORAGE = 0x3a,
  TSS_FORMAT_AND_INITIALIZE_SD_CARD = 0x3b,
  TSS_BEGIN_DATA_LOGGING_SESSION = 0x3c,
  TSS_END_DATA_LOGGING_SESSION = 0x3d,
  TSS_SET_CLOCK_VALUES = 0x3e,
  TSS_GET_CLOCK_VALUES = 0x3f,
  TSS_SET_STREAMING_SLOTS = 0x50,
  TSS_GET_STREAMING_SLOTS = 0x51,
  TSS_SET_STREAMING_TIMING = 0x52,
  TSS_GET_STREAMING_TIMING = 0x53,
  TSS_GET_STREAMING_BATCH = 0x54,
  TSS_START_STREAMING = 0x55,
  TSS_STOP_STREAMING = 0x56,
  TSS_UPDATE_CURRENT_TIMESTAMP = 0x5f,
  TSS_TARE_WITH_CURRENT_ORIENTATION = 0x60,
  TSS_TARE_WITH_QUATERNION = 0x61,
  TSS_TARE_WITH_ROTATION_MATRIX = 0x62,
  TSS_SET_STATIC_ACCELEROMETER_TRUST_VALUE = 0x63,
  TSS_SET_CONFIDENCE_ACCELEROMETER_TRUST_VALUES = 0x64,
  TSS_SET_STATIC_COMPASS_TRUST_VALUE = 0x65,
  TSS_SET_CONFIDENCE_COMPASS_TRUST_VALUES = 0x66,
  TSS_SET_DESIRED_UPDATE_RATE = 0x67,
  TSS_SET_MULTI_REFERENCE_VECTORS_WITH_CURRENT_ORIENTATION = 0x68,
  TSS_SET_REFERENCE_VECTOR_MODE = 0x69,
  TSS_SET_OVERSAMPLE_RATE = 0x6a,
  TSS_SET_GYROSCOPE_ENABLED = 0x6b,
  TSS_SET_ACCELEROMETER_ENABLED = 0x6c,
  TSS_SET_COMPASS_ENABLED = 0x6d,
  TSS_RESET_MULTI_REFERENCE_VECTORS_TO_ZERO = 0x6e,
  TSS_SET_MULTI_REFERENCE_TABLE_RESOLUTION = 0x6f,
  TSS_SET_COMPASS_MULTI_REFERENCE_VECTOR = 0x70,
  TSS_SET_COMPASS_MULTI_REFERENCE_CHECK_VECTOR = 0x71,
  TSS_SET_ACCELEROMETER_MULTI_REFERENCE_VECTOR = 0x72,
  TSS_SET_ACCELEROMETER_MULTI_REFERENCE_CHECK_VECTOR = 0x73,
  TSS_SET_AXIS_DIRECTIONS = 0x74,
  TSS_SET_RUNNING_AVERAGE_PERCENT = 0x75,
  TSS_SET_COMPASS_REFERENCE_VECTOR = 0x76,
  TSS_SET_ACCELEROMETER_REFERENCE_VECTOR = 0x77,
  TSS_RESET_KALMAN_FILTER = 0x78,
  TSS_SET_ACCELEROMETER_RANGE = 0x79,
  TSS_SET_MULTI_REFERENCE_WEIGHT_POWER = 0x7a,
  TSS_SET_FILTER_MODE = 0x7b,
  TSS_SET_RUNNING_AVERAGE_MODE = 0x7c,
  TSS_SET_GYROSCOPE_RANGE = 0x7d,
  TSS_SET_COMPASS_RANGE = 0x7e,
  TSS_GET_TARE_AS_QUATERNION = 0x80,
  TSS_GET_TARE_AS_ROTATION_MATRIX = 0x81,
  TSS_GET_ACCELEROMETER_TRUST_VALUES = 0x82,
  TSS_GET_COMPASS_TRUST_VALUES = 0x83,
  TSS_GET_CURRENT_UPDATE_RATE = 0x84,
  TSS_GET_COMPASS_REFERENCE_VECTOR = 0x85,
  TSS_GET_ACCELEROMETER_REFERENCE_VECTOR = 0x86,
  TSS_GET_REFERENCE_VECTOR_MODE = 0x87,
  TSS_GET_COMPASS_MULTI_REFERENCE_VECTOR = 0x88,
  TSS_GET_COMPASS_MULTI_REFERENCE_CHECK_VECTOR = 0x89,
  TSS_GET_ACCELEROMETER_MULTI_REFERENCE_VECTOR = 0x8a,
  TSS_GET_ACCELEROMETER_MULTI_REFERENCE_CHECK_VECTOR = 0x8b,
  TSS_GET_GYROSCOPE_ENABLED_STATE = 0x8c,
  TSS_GET_ACCELEROMETER_ENABLED_STATE = 0x8d,
  TSS_GET_COMPASS_ENABLED_STATE = 0x8e,
  TSS_GET_AXIS_DIRECTIONS = 0x8f,
  TSS_GET_OVERSAMPLE_RATE = 0x90,
  TSS_GET_RUNNING_AVERAGE_PERCENT = 0x91,
  TSS_GET_DESIRED_UPDATE_RATE = 0x92,
  TSS_GET_ACCELEROMETER_RANGE = 0x94,
  TSS_GET_MULTI_REFERENCE_MODE_POWER_WEIGHT = 0x95,
  TSS_GET_MULTI_REFERENCE_RESOLUTION = 0x96,
  TSS_GET_NUMBER_OF_MULTI_REFERENCE_CELLS = 0x97,
  TSS_GET_FILTER_MODE = 0x98,
  TSS_GET_RUNNING_AVERAGE_MODE = 0x99,
  TSS_GET_GYROSCOPE_RANGE = 0x9a,
  TSS_GET_COMPASS_RANGE = 0x9b,
  TSS_GET_EULER_ANGLE_DECOMPOSITION_ORDER = 0x9c,
  TSS_GET_MAGNETORESISTIVE_THRESHOLD = 0X9d,
  TSS_GET_ACCELEROMETER_RESISTANCE_THRESHOLD = 0x9e,
  TSS_GET_OFFSET_ORIENTATION_AS_QUATERNION = 0x9f,
  TSS_SET_COMPASS_CALIBRATION_COEFFICIENTS = 0xa0,
  TSS_SET_ACCELEROMETER_CALIBRATION_COEFFICIENTS = 0xa1,
  TSS_GET_COMPASS_CALIBRATION_COEFFICIENTS = 0xa2,
  TSS_GET_ACCELEROMETER_CALIBRATION_COEFFICIENTS = 0xa3,
  TSS_GET_GYROSCOPE_CALIBRATION_COEFFICIENTS = 0xa4,
  TSS_BEGIN_GYROSCOPE_AUTO_CALIBRATION = 0xa5,
  TSS_SET_GYROSCOPE_CALIBRATION_COEFFICIENTS = 0xa6,
  TSS_SET_CALIBRATION_MODE = 0xa9,
  TSS_GET_CALIBRATION_MODE = 0xaa,
  TSS_SET_ORTHO_CALIBRATION_DATA_POINT_FROM_CURRENT_ORIENTATION = 0xab,
  TSS_SET_ORTHO_CALIBRATION_DATA_POINT_FROM_VECTOR = 0xac,
  TSS_GET_ORTHO_CALIBRATION_DATA_POINT = 0xad,
  TSS_PERFORM_ORTHO_CALIBRATION = 0xae,
  TSS_CLEAR_ORTHO_CALIBRATION_DATA = 0xaf,
  TSS_SET_WIRELESS_STREAMING_AUTO_FLUSH_MODE = 0xb0,
  TSS_GET_WIRELESS_STREAMING_AUTO_FLUSH_MODE = 0xb1,
  TSS_SET_WIRELESS_STREAMING_MANUAL_FLUSH_BITFIELD = 0xb2,
  TSS_GET_WIRELESS_STREAMING_MANUAL_FLUSH_BITFIELD = 0xb3,
  TSS_GET_MANUAL_FLUSH_SINGLE = 0xb4,
  TSS_GET_MANUAL_FLUSH_BULK = 0xb5,
  TSS_BROADCAST_SYNCHRONIZATION_PULSE = 0xb6,
  TSS_GET_RECEPTION_BITFIELD = 0xb7,
  TSS_GET_WIRELESS_PAN_ID = 0xc0,
  TSS_SET_WIRELESS_PAN_ID = 0xc1,
  TSS_GET_WIRELESS_CHANNEL = 0xc2,
  TSS_SET_WIRELESS_CHANNEL = 0xc3,
  TSS_SET_LED_MODE = 0xc4,
  TSS_COMMIT_WIRELESS_SETTINGS = 0xc5,
  TSS_GET_WIRELESS_ADDRESS = 0xc6,
  TSS_GET_LED_MODE = 0xc8,
  TSS_GET_SERIAL_NUMBER_AT_LOGICAL_ID = 0xd0,
  TSS_SET_SERIAL_NUMBER_AT_LOGICAL_ID = 0xd1,
  TSS_GET_WIRELESS_CHANNEL_NOISE_LEVELS = 0xd2,
  TSS_SET_WIRELESS_RETRIES = 0xd3,
  TSS_GET_WIRELESS_RETRIES = 0xd4,
  TSS_GET_WIRELESS_SLOTS_OPEN = 0xd5,
  TSS_GET_SIGNAL_STRENGTH = 0xd6,
  TSS_SET_WIRELESS_HID_UPDATE_RATE = 0xd7,
  TSS_GET_WIRELESS_HID_UPDATE_RATE = 0xd8,
  TSS_SET_WIRELESS_HID_ASYNCHRONOUS_MODE = 0xd9,
  TSS_GET_WIRELESS_HID_ASYNCHRONOUS_MODE = 0xda,
  TSS_SET_WIRELESS_RESPONSE_HEADER_BITFIELD = 0xdb,
  TSS_GET_WIRELESS_RESPONSE_HEADER_BITFIELD = 0xdc,
  TSS_SET_WIRED_RESPONSE_HEADER_BITFIELD = 0xdd,
  TSS_GET_WIRED_RESPONSE_HEADER_BITFIELD = 0xde,
  TSS_GET_FIRMWARE_VERSION_STRING = 0xdf,
  TSS_RESTORE_FACTORY_SETTINGS = 0xe0,
  TSS_COMMIT_SETTINGS = 0xe1,
  TSS_SOFTWARE_RESET = 0xe2,
  TSS_SET_SLEEP_MODE = 0xe3,
  TSS_GET_SLEEP_MODE = 0xe4,
  TSS_ENTER_BOOTLOADER_MODE = 0xe5,
  TSS_GET_HARDWARE_VERSION_STRING = 0xe6,
  TSS_SET_UART_BAUD_RATE = 0xe7,
  TSS_GET_UART_BAUD_RATE = 0xe8,
  TSS_SET_USB_MODE = 0xe9,
  TSS_GET_USB_MODE = 0xea,
  TSS_GET_SERIAL_NUMBER = 0xed,
  TSS_SET_LED_COLOR = 0xee,
  TSS_GET_LED_COLOR = 0xef,
  TSS_SET_JOYSTICK_LOGICAL_ID = 0xf0,
  TSS_SET_MOUSE_LOGICAL_ID = 0xf1,
  TSS_GET_JOYSTICK_LOGICAL_ID = 0xf2,
  TSS_GET_MOUSE_LOGICAL_ID = 0xf3,
  TSS_SET_CONTROL_MODE = 0xf4,
  TSS_SET_CONTROL_DATA = 0xf5,
  TSS_GET_CONTROL_MODE = 0xf6,
  TSS_GET_CONTROL_DATA = 0xf7,
  TSS_SET_BUTTON_GYRO_DISABLE_LENGTH = 0xf8,
  TSS_GET_BUTTON_GYRO_DISABLE_LENGTH = 0xf9,
  TSS_SET_MOUSE_ABSOLUTE_RELATIVE_MODE = 0xfb,
  TSS_GET_MOUSE_ABSOLUTE_RELATIVE_MODE = 0xfc,
  TSS_SET_JOYSTICK_AND_MOUSE_PRESENT_REMOVED = 0xfd,
  TSS_GET_JOYSTICK_AND_MOUSE_PRESENT_REMOVED = 0xfe,
  TSS_SET_JOYSTICK_ENABLED = 0x100,
  TSS_SET_MOUSE_ENABLED = 0x101,
  TSS_GET_JOYSTICK_ENABLED = 0x102,
  TSS_GET_MOUSE_ENABLED = 0x103
} TSS_Command_Enum;

//UNUSED

HAL_StatusTypeDef getCorrectedAccelerometerVector (float3D* data);
HAL_StatusTypeDef getUntaredEulerAngles (float3D* data);

#endif /* IMU_IMU_H_ */