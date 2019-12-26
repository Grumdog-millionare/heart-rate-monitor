void devMAX30105init(const uint8_t i2cAddress);

WarpStatus writeSensorRegisterMAX30105(uint8_t deviceRegister,
									   uint8_t payload);

WarpStatus configureSensorMAX30105();

WarpStatus readSensorRegisterMAX30105(uint8_t deviceRegister, int numberOfBytes);

WarpStatus readSample(uint16_t *data, uint32_t *sample);