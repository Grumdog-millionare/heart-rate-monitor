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

WarpStatus
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
		return kWarpStatusBadDeviceCommand;
	}
	}
	SEGGER_RTT_printf(0, "TEST 3.1\n", 0);

	i2c_device_t slave =
		{
			.address = deviceMAX30105State.i2cAddress,
			.baudRate_kbps = gWarpI2cBaudRateKbps};
	SEGGER_RTT_printf(0, "TEST 3.2, 0x%02x, 0x%02x \n", slave.address);

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

	SEGGER_RTT_printf(0, "TEST 3.3\n", 0);
	if (status != kStatus_I2C_Success)
	{
		return kWarpStatusDeviceCommunicationFailed;
	}

	return kWarpStatusOK;
}

WarpStatus
configureSensorMAX30105()
{
	WarpStatus
		i2cWriteStatus_FIFO_CONFIG,
		i2cWriteStatus_SPO2_CONFIG,
		i2cWriteStatus_LED1_CONFIG,
		i2cWriteStatus_LED2_CONFIG,
		i2cWriteStatus_LED3_CONFIG,
		i2cWriteStatus_LED_PROX_CONFIG,
		i2cWriteStatus_MODE_CONFIG,
		i2cWriteStatus_MULTI_LED_MODE_CONTROL_CONFIG;

	SEGGER_RTT_printf(0, "TEST 2\n", 0);

	// SET FIFO: Sample averaging = 4, FIFO rolls on full = True
	i2cWriteStatus_FIFO_CONFIG = writeSensorRegisterMAX30105(FIFO_CONFIG, 0x50);
	SEGGER_RTT_printf(0, "TEST 4 ", 0);

	// SET SPO2: ADC range = 16384, Sample rate = 400 Hz, Pulse width = 215 us
	i2cWriteStatus_SPO2_CONFIG = writeSensorRegisterMAX30105(SPO2_CONFIG, 0x6E);
	SEGGER_RTT_printf(0, "TEST 5 ", 0);

	// SET LED1 PULSE AMPLITUDE: Current level = 0.2 mA
	i2cWriteStatus_LED1_CONFIG = writeSensorRegisterMAX30105(LED1_PULSE_AMPLITUDE, 0x01);
	SEGGER_RTT_printf(0, "TEST 6 ", 0);

	// SET LED2 PULSE AMPLITUDE: Current level = 12.5 mA
	i2cWriteStatus_LED2_CONFIG = writeSensorRegisterMAX30105(LED2_PULSE_AMPLITUDE, 0x3F);
	SEGGER_RTT_printf(0, "TEST 7 ", 0);

	// SET LED3 PULSE AMPLITUDE: Current level = 0.0 mA
	i2cWriteStatus_LED3_CONFIG = writeSensorRegisterMAX30105(LED3_PULSE_AMPLITUDE, 0x00);
	SEGGER_RTT_printf(0, "TEST 8 ", 0);

	// SET LED PROXIMITY MODE PULSE AMPLITUDE: Current level = 6.4mA
	i2cWriteStatus_LED_PROX_CONFIG = writeSensorRegisterMAX30105(PROX_MODE_LED_PULSE_AMPLITUDE, 0x1F);
	SEGGER_RTT_printf(0, "TEST 9 ", 0);

	// SET MODE: Particle sensing mode using 2 LEDs
	i2cWriteStatus_MODE_CONFIG = writeSensorRegisterMAX30105(MODE_CONFIG, 0x03);
	SEGGER_RTT_printf(0, "TEST 10 ", 0);

	// SET MULTI LED MODE CONTROL: Particle sensing mode using 2 LEDs
	i2cWriteStatus_MULTI_LED_MODE_CONTROL_CONFIG = writeSensorRegisterMAX30105(MULTI_LED_MODE_CONTROL_CONFIG, 0x21);
	SEGGER_RTT_printf(0, "TEST 11 ", 0);

	return (i2cWriteStatus_FIFO_CONFIG |
			i2cWriteStatus_SPO2_CONFIG |
			i2cWriteStatus_LED1_CONFIG |
			i2cWriteStatus_LED2_CONFIG |
			i2cWriteStatus_LED3_CONFIG |
			i2cWriteStatus_LED_PROX_CONFIG |
			i2cWriteStatus_MODE_CONFIG |
			i2cWriteStatus_MULTI_LED_MODE_CONTROL_CONFIG);
}

WarpStatus
readSensorRegisterMAX30105(uint8_t deviceRegister, int numberOfBytes)
{
	uint8_t cmdBuf[1] = {0xFF};
	i2c_status_t status;

	USED(numberOfBytes);
	switch (deviceRegister)
	{
	case 0x07:
	{
		/* OK */
		break;
	}

	default:
	{
		return kWarpStatusBadDeviceCommand;
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
		return kWarpStatusDeviceCommunicationFailed;
	}

	return kWarpStatusOK;
}

WarpStatus readSample(uint16_t *data, uint32_t *sample)
{

	WarpStatus i2cReadStatus;

	i2cReadStatus = readSensorRegisterMAX30105(FIFO_DATA, 6 /* numberOfBytes */);

	if (i2cReadStatus != kWarpStatusOK)
	{
		return kWarpStatusDeviceCommunicationFailed;
	}

	for (int i = 0; i < 6; i++)
	{
		data[i] = deviceMAX30105State.i2cBuffer[i];
	}

	for (int i = 0; i < 2; i++)
	{
		sample[i] = (data[i] << 16) | (data[i + 1] << 8) | (data[i + 2]);
	}

	return kWarpStatusOK;
}