CommStatus writeSensorRegisterMAX30105(uint8_t deviceRegister,
									   uint8_t payload);

CommStatus devMAX30105init(const uint8_t i2cAddress);

CommStatus readSensorRegisterMAX30105(uint8_t deviceRegister, int numberOfBytes);

CommStatus readNextSample(uint16_t *sample);