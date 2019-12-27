#include <stdlib.h>

#include "fsl_misc_utilities.h"
#include "fsl_device_registers.h"
#include "fsl_i2c_master_driver.h"
#include "fsl_spi_master_driver.h"
#include "fsl_rtc_driver.h"
#include "fsl_clock_manager.h"
#include "fsl_power_manager.h"
#include "fsl_mcglite_hal.h"
#include "fsl_port_hal.h"

#include "gpio_pins.h"
#include "SEGGER_RTT.h"
#include "warp.h"

extern volatile WarpI2CDeviceState deviceMAX30105State;
extern volatile uint32_t gWarpI2cBaudRateKbps;
extern volatile uint32_t gWarpI2cTimeoutMilliseconds;
extern volatile uint32_t gWarpSupplySettlingDelayMilliseconds;

void devMAX30105init(const uint8_t i2cAddress)
{
	deviceMAX30105State.i2cAddress = i2cAddress;
	return;
}

CommStatus
writeSensorRegisterMAX30105(uint8_t deviceRegister, uint8_t payload)
{
	uint8_t payloadByte[1], commandByte[1];
	i2c_status_t status;

	switch (deviceRegister)
	{
	case 0x04:
	case 0x05:
	case 0x06:
	case 0x08:
	case 0x09:
	case 0x0A:
	case 0x0C:
	case 0x0D:
	case 0x0E:
	case 0x10:
	case 0x11:
	{
		/* OK */
		break;
	}

	default:
	{
		return CommStatusBadDeviceCommand;
	}
	}

	i2c_device_t slave =
		{
			.address = deviceMAX30105State.i2cAddress,
			.baudRate_kbps = gWarpI2cBaudRateKbps};

	commandByte[0] = deviceRegister;
	payloadByte[0] = payload;
	status = I2C_DRV_MasterSendDataBlocking(
		0 /* I2C instance */,
		&slave,
		commandByte,
		1,
		payloadByte,
		1,
		gWarpI2cTimeoutMilliseconds);

	if (status != kStatus_I2C_Success)
	{
		return CommStatusDeviceCommunicationFailed;
	}

	return CommStatusOK;
}

CommStatus
configureSensorMAX30105()
{
	CommStatus
		i2cWriteStatus_FIFO_CONFIG,
		i2cWriteStatus_SPO2_CONFIG,
		i2cWriteStatus_LED1_CONFIG,
		i2cWriteStatus_LED2_CONFIG,
		i2cWriteStatus_LED3_CONFIG,
		i2cWriteStatus_LED_PROX_CONFIG,
		i2cWriteStatus_MODE_CONFIG,
		i2cWriteStatus_MULTI_LED_MODE_CONTROL_CONFIG;

	// SET FIFO: Sample averaging = 4, FIFO rolls on full = True
	i2cWriteStatus_FIFO_CONFIG = writeSensorRegisterMAX30105(FIFO_CONFIG, 0x50);

	// SET SPO2: ADC range = 16384, Sample rate = 400 Hz, Pulse width = 215 us
	i2cWriteStatus_SPO2_CONFIG = writeSensorRegisterMAX30105(SPO2_CONFIG, 0x6E);

	// SET LED1 PULSE AMPLITUDE: Current level = 0.2 mA
	i2cWriteStatus_LED1_CONFIG = writeSensorRegisterMAX30105(LED1_PULSE_AMPLITUDE, 0x01);

	// SET LED2 PULSE AMPLITUDE: Current level = 12.5 mA
	i2cWriteStatus_LED2_CONFIG = writeSensorRegisterMAX30105(LED2_PULSE_AMPLITUDE, 0x3F);

	// SET LED3 PULSE AMPLITUDE: Current level = 0.0 mA
	i2cWriteStatus_LED3_CONFIG = writeSensorRegisterMAX30105(LED3_PULSE_AMPLITUDE, 0x00);

	// SET LED PROXIMITY MODE PULSE AMPLITUDE: Current level = 6.4mA
	i2cWriteStatus_LED_PROX_CONFIG = writeSensorRegisterMAX30105(PROX_MODE_LED_PULSE_AMPLITUDE, 0x1F);

	// SET MODE: Particle sensing mode using 2 LEDs
	i2cWriteStatus_MODE_CONFIG = writeSensorRegisterMAX30105(MODE_CONFIG, 0x03);

	// SET MULTI LED MODE CONTROL: Particle sensing mode using 2 LEDs
	i2cWriteStatus_MULTI_LED_MODE_CONTROL_CONFIG = writeSensorRegisterMAX30105(MULTI_LED_MODE_CONTROL_CONFIG, 0x21);

	return (i2cWriteStatus_FIFO_CONFIG |
			i2cWriteStatus_SPO2_CONFIG |
			i2cWriteStatus_LED1_CONFIG |
			i2cWriteStatus_LED2_CONFIG |
			i2cWriteStatus_LED3_CONFIG |
			i2cWriteStatus_LED_PROX_CONFIG |
			i2cWriteStatus_MODE_CONFIG |
			i2cWriteStatus_MULTI_LED_MODE_CONTROL_CONFIG);
}

CommStatus
readSensorRegisterMAX30105(uint8_t deviceRegister, int numberOfBytes)
{
	uint8_t cmdBuf[1] = {0xFF};
	i2c_status_t status;

	switch (deviceRegister)
	{
	case 0x07:
	{
		/* OK */
		break;
	}

	default:
	{
		return CommStatusBadDeviceCommand;
	}
	}

	i2c_device_t slave =
		{
			.address = deviceMAX30105State.i2cAddress,
			.baudRate_kbps = gWarpI2cBaudRateKbps};

	cmdBuf[0] = deviceRegister;

	status = I2C_DRV_MasterReceiveDataBlocking(
		0 /* I2C peripheral instance */,
		&slave,
		cmdBuf,
		1,
		(uint8_t *)deviceMAX30105State.i2cBuffer,
		numberOfBytes,
		gWarpI2cTimeoutMilliseconds);

	if (status != kStatus_I2C_Success)
	{
		return CommStatusDeviceCommunicationFailed;
	}

	return CommStatusOK;
}

SamplingStatus readLatestSample(uint32_t *sample)
{

	// WarpStatus i2cReadStatus_FIFO_READ, i2cReadStatus_FIFO_WRITE, i2cReadStatus_FIFO_DATA;

	// // Read READ pointer
	// i2cReadStatus_FIFO_READ = readSensorRegisterMAX30105(FIFO_READ, 1 /* numberOfBytes */);
	// uint8_t read_pointer = deviceMAX30105State.i2cBuffer[0];

	// // Read WRITE pointer
	// i2cReadStatus_FIFO_WRITE = readSensorRegisterMAX30105(FIFO_WRITE, 1 /* numberOfBytes */);
	// uint8_t write_pointer = deviceMAX30105State.i2cBuffer[0];

	// if (read_pointer == write_pointer)
	// {
	// 	return kSampleNotUpdated;
	// }

	// uint8_t sample_count = write_pointer - read_pointer;

	// if (sample_count < 0)
	// {
	// 	sample_count = 32;
	// }

	// uint8_t byte_count = sample_count * 3 * 2; // 3 bytes for each of the two channels (RED and IR)

	// uint8_t data[byte_count];

	// i2cReadStatus_FIFO_DATA = readSensorRegisterMAX30105(FIFO_DATA, byte_count /* numberOfBytes */);

	// if ((i2cReadStatus_FIFO_READ | i2cReadStatus_FIFO_WRITE | i2cReadStatus_FIFO_DATA) != kWarpStatusOK)
	// {
	// 	return kWarpStatusDeviceCommunicationFailed;
	// }

	// for (int i = 0; i < byte_count; i++)
	// {
	// 	data[i] = deviceMAX30105State.i2cBuffer[i];
	// }

	// int last_red_sample_index = byte_count - 6;
	// int last_ir_sample_index = byte_count - 3;
	// sample[0] = (data[last_red_sample_index] << 16) | (data[last_red_sample_index + 1] << 8) | (data[last_red_sample_index + 2]); // RED channel
	// sample[1] = (data[last_ir_sample_index] << 16) | (data[last_ir_sample_index + 1] << 8) | (data[last_ir_sample_index + 2]);	// IR channel

	CommStatus i2cReadStatus = readSensorRegisterMAX30105(FIFO_DATA, 6 /* numberOfBytes */);
	uint8_t data[6];
	for (int i = 0; i < 6; i++)
	{
		data[i] = deviceMAX30105State.i2cBuffer[i];
	}

	sample[0] = (data[0] << 16) | (data[1] << 8) | (data[2]);
	sample[1] = (data[3] << 16) | (data[4] << 8) | (data[5]);

	return SampleOK;
}