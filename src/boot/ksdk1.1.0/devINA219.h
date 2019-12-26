#ifndef WARP_BUILD_ENABLE_DEVINA219
#define WARP_BUILD_ENABLE_DEVINA219
#endif

void initINA219(const uint8_t i2cAddress, WarpI2CDeviceState volatile *deviceStatePointer);
WarpStatus writeSensorRegisterINA219(uint8_t deviceRegister,
									 uint8_t payloadByte_MSB,
									 uint8_t payloadByte_LSB,
									 uint16_t menuI2cPullupValue);
WarpStatus configureSensorINA219(uint8_t payload_Configuration_MSB, uint8_t payload_Configuration_LSB, uint8_t payload_Calibration_MSB, uint8_t payload_Calibration_LSB, uint16_t menuI2cPullupValue);
WarpStatus readSensorRegisterINA219(uint8_t deviceRegister, int numberOfBytes);
void printSensorDataINA219(bool hexModeFlag);