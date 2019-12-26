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

extern volatile WarpI2CDeviceState deviceINA219State;
extern volatile uint32_t gWarpI2cBaudRateKbps;
extern volatile uint32_t gWarpI2cTimeoutMilliseconds;
extern volatile uint32_t gWarpSupplySettlingDelayMilliseconds;

void initINA219(const uint8_t i2cAddress, WarpI2CDeviceState volatile *deviceStatePointer)
{
	deviceStatePointer->i2cAddress = i2cAddress;
	deviceStatePointer->signalType = (kWarpTypeMaskCurrent);
	return;
}

WarpStatus
writeSensorRegisterINA219(uint8_t deviceRegister, uint8_t payload_MSB, uint8_t payload_LSB, uint16_t menuI2cPullupValue)
{
	uint8_t payloadBytes[2], commandByte[1];
	i2c_status_t status;

	switch (deviceRegister)
	{
	case 0x00:
	case 0x05:
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
			.address = deviceINA219State.i2cAddress,
			.baudRate_kbps = gWarpI2cBaudRateKbps};

	commandByte[0] = deviceRegister;
	payloadBytes[0] = payload_MSB;
	payloadBytes[1] = payload_LSB;
	status = I2C_DRV_MasterSendDataBlocking(
		0 /* I2C instance */,
		&slave,
		commandByte,
		1,
		payloadBytes,
		2,
		gWarpI2cTimeoutMilliseconds);
	if (status != kStatus_I2C_Success)
	{
		return kWarpStatusDeviceCommunicationFailed;
	}

	return kWarpStatusOK;
}

WarpStatus
configureSensorINA219(uint8_t payload_Configuration_MSB, uint8_t payload_Configuration_LSB, uint8_t payload_Calibration_MSB, uint8_t payload_Calibration_LSB, uint16_t menuI2cPullupValue)
{
	WarpStatus i2cWriteStatus1, i2cWriteStatus2;

	i2cWriteStatus1 = writeSensorRegisterINA219(kWarpSensorOutputRegisterINA219Configuration /* register address Configuration */,
												payload_Configuration_MSB,
												payload_Configuration_LSB,
												menuI2cPullupValue);

	i2cWriteStatus2 = writeSensorRegisterINA219(kWarpSensorOutputRegisterINA219Calibration /* register address Calibration */,
												payload_Calibration_MSB,
												payload_Calibration_LSB,
												menuI2cPullupValue);

	return (i2cWriteStatus1 | i2cWriteStatus2);
}

WarpStatus
readSensorRegisterINA219(uint8_t deviceRegister, int numberOfBytes)
{
	uint8_t cmdBuf[1] = {0xFF};
	i2c_status_t status;

	USED(numberOfBytes);
	switch (deviceRegister)
	{
	case 0x00:
	case 0x01:
	case 0x02:
	case 0x03:
	case 0x04:
	case 0x05:
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
			.address = deviceINA219State.i2cAddress,
			.baudRate_kbps = gWarpI2cBaudRateKbps};

	cmdBuf[0] = deviceRegister;

	status = I2C_DRV_MasterReceiveDataBlocking(
		0 /* I2C peripheral instance */,
		&slave,
		cmdBuf,
		1,
		(uint8_t *)deviceINA219State.i2cBuffer,
		numberOfBytes,
		gWarpI2cTimeoutMilliseconds);

	if (status != kStatus_I2C_Success)
	{
		return kWarpStatusDeviceCommunicationFailed;
	}

	return kWarpStatusOK;
}

void printSensorDataINA219(bool hexModeFlag)
{
	uint16_t readSensorRegisterValueLSB;
	uint16_t readSensorRegisterValueMSB;
	int16_t readSensorRegisterValueCombined;
	WarpStatus i2cReadStatus;

	i2cReadStatus = readSensorRegisterINA219(kWarpSensorOutputRegisterINA219Current, 2 /* numberOfBytes */);
	readSensorRegisterValueMSB = deviceINA219State.i2cBuffer[0];
	readSensorRegisterValueLSB = deviceINA219State.i2cBuffer[1];
	readSensorRegisterValueCombined = ((readSensorRegisterValueMSB & 0xFF) << 8) | readSensorRegisterValueLSB;

	if (i2cReadStatus != kWarpStatusOK)
	{
		SEGGER_RTT_WriteString(0, " ----,");
	}
	else
	{
		if (hexModeFlag)
		{
			SEGGER_RTT_printf(0, " 0x%02x 0x%02x,", readSensorRegisterValueMSB, readSensorRegisterValueLSB);
		}
		else
		{
			SEGGER_RTT_printf(0, " %d,", readSensorRegisterValueCombined);
		}
	}
}