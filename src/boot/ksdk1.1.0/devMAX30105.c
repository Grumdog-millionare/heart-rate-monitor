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

extern const uint32_t THRESHOLD_UP;

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
	case 0x02:
	case 0x03:
	case 0x04:
	case 0x05:
	case 0x06:
	case 0x07:
	case 0x08:
	case 0x09:
	case 0x0A:
	case 0x0C:
	case 0x0D:
	case 0x0E:
	case 0x10:
	case 0x11:
	case 0x12:
	case 0x30:
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
		i2cWriteStatus_INTERRUPT_ENABLE_1,
		i2cWriteStatus_PROXIMITY_THRESHOLD,
		i2cWriteStatus_FIFO_CONFIG,
		i2cWriteStatus_SPO2_CONFIG,
		i2cWriteStatus_LED1_CONFIG,
		i2cWriteStatus_LED2_CONFIG,
		i2cWriteStatus_LED3_CONFIG,
		i2cWriteStatus_LED_PROX_CONFIG,
		i2cWriteStatus_MODE_CONFIG,
		i2cWriteStatus_MULTI_LED_MODE_CONTROL_CONFIG;

	// SET INTERRUPT ENABLE: Data ready interrupt = Off, Proximity interrupt = On
	i2cWriteStatus_INTERRUPT_ENABLE_1 = writeSensorRegisterMAX30105(INTERRUPT_ENABLE_1, 0x10);

	// SET PROX THRESHOLD: Data ready interrupt = Off, Proximity interrupt = On
	i2cWriteStatus_PROXIMITY_THRESHOLD = writeSensorRegisterMAX30105(PROXIMITY_THRESHOLD, (THRESHOLD_UP >> 10));

	// SET FIFO: Sample averaging = 8, FIFO rolls on full = True
	i2cWriteStatus_FIFO_CONFIG = writeSensorRegisterMAX30105(FIFO_CONFIG, 0x70);

	// SET SPO2: ADC range = 16384, Sample rate = 400 Hz, Pulse width = 215 us
	i2cWriteStatus_SPO2_CONFIG = writeSensorRegisterMAX30105(SPO2_CONFIG, 0x6E);

	// SET LED1 (RED) PULSE AMPLITUDE: Current level = 0.4 mA (0x02)
	i2cWriteStatus_LED1_CONFIG = writeSensorRegisterMAX30105(LED1_PULSE_AMPLITUDE, 0x02);

	// SET LED2 (IR) PULSE AMPLITUDE: Current level = 12.5 mA (0x3F)
	i2cWriteStatus_LED2_CONFIG = writeSensorRegisterMAX30105(LED2_PULSE_AMPLITUDE, 0x3F);

	// SET LED3 (GREEN) PULSE AMPLITUDE: Current level = 0.0 mA (0x00)
	i2cWriteStatus_LED3_CONFIG = writeSensorRegisterMAX30105(LED3_PULSE_AMPLITUDE, 0x00);

	// SET LED PROXIMITY MODE PULSE AMPLITUDE: Current level = 0.4 mA (0x02)
	i2cWriteStatus_LED_PROX_CONFIG = writeSensorRegisterMAX30105(PROX_MODE_LED_PULSE_AMPLITUDE, 0x02);

	// SET MODE: Particle sensing mode using 2 LEDs
	i2cWriteStatus_MODE_CONFIG = writeSensorRegisterMAX30105(MODE_CONFIG, 0x03);

	// SET MULTI LED MODE CONTROL: Particle sensing mode using 2 LEDs
	i2cWriteStatus_MULTI_LED_MODE_CONTROL_CONFIG = writeSensorRegisterMAX30105(MULTI_LED_MODE_CONTROL_CONFIG, 0x21);

	return (i2cWriteStatus_INTERRUPT_ENABLE_1 |
			i2cWriteStatus_PROXIMITY_THRESHOLD |
			i2cWriteStatus_FIFO_CONFIG |
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
	case 0x00:
	case 0x01:
	case 0x02:
	case 0x03:
	case 0x04:
	case 0x05:
	case 0x06:
	case 0x07:
	case 0x08:
	case 0x09:
	case 0x0A:
	case 0x0C:
	case 0x0D:
	case 0x0E:
	case 0x10:
	case 0x11:
	case 0x12:
	case 0x1F:
	case 0x20:
	case 0x21:
	case 0x30:
	case 0xFE:
	case 0xFF:

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

SamplingStatus readNextSample(uint32_t *sample)
{

	CommStatus i2cReadStatus_FIFO_READ, i2cReadStatus_FIFO_WRITE, i2cReadStatus_FIFO_DATA;

	// Read READ pointer
	i2cReadStatus_FIFO_READ = readSensorRegisterMAX30105(FIFO_READ, 1 /* numberOfBytes */);
	uint8_t read_pointer = deviceMAX30105State.i2cBuffer[0];

	// Read WRITE pointer
	i2cReadStatus_FIFO_WRITE = readSensorRegisterMAX30105(FIFO_WRITE, 1 /* numberOfBytes */);
	uint8_t write_pointer = deviceMAX30105State.i2cBuffer[0];

	if ((read_pointer == write_pointer) && (read_pointer != 0))
	{
		return SampleNotUpdated;
	}

	i2cReadStatus_FIFO_DATA = readSensorRegisterMAX30105(FIFO_DATA, 6 /* numberOfBytes */);

	if ((i2cReadStatus_FIFO_READ | i2cReadStatus_FIFO_WRITE | i2cReadStatus_FIFO_DATA) != CommStatusOK)
	{
		return SamplingFailed;
	}

	uint8_t data[6];
	for (int i = 0; i < 6; i++)
	{
		data[i] = deviceMAX30105State.i2cBuffer[i];
	}

	*sample = (data[3] << 16) | (data[4] << 8) | (data[5]); // IR channel

	return SampleOK;
}