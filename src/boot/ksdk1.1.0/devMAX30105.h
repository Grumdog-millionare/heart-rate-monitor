void devMAX30105init(const uint8_t i2cAddress);

CommStatus writeSensorRegisterMAX30105(uint8_t deviceRegister,
									   uint8_t payload);

CommStatus configureSensorMAX30105();

CommStatus readSensorRegisterMAX30105(uint8_t deviceRegister, int numberOfBytes);

CommStatus readNextSample(uint32_t * sample);