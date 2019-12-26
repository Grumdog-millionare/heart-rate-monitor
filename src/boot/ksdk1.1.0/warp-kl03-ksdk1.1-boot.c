/*
	Authored 2016-2018. Phillip Stanley-Marbell.
	
	Additional contributions, 2018: Jan Heck, Chatura Samarakoon, Youchao Wang, Sam Willis.

	All rights reserved.

	Redistribution and use in source and binary forms, with or without
	modification, are permitted provided that the following conditions
	are met:

	*	Redistributions of source code must retain the above
		copyright notice, this list of conditions and the following
		disclaimer.

	*	Redistributions in binary form must reproduce the above
		copyright notice, this list of conditions and the following
		disclaimer in the documentation and/or other materials
		provided with the distribution.

	*	Neither the name of the author nor the names of its
		contributors may be used to endorse or promote products
		derived from this software without specific prior written
		permission.

	THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
	"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
	LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
	FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
	COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
	INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
	BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
	LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
	CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
	LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
	ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
	POSSIBILITY OF SUCH DAMAGE.
*/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "fsl_misc_utilities.h"
#include "fsl_device_registers.h"
#include "fsl_i2c_master_driver.h"
#include "fsl_spi_master_driver.h"
#include "fsl_rtc_driver.h"
#include "fsl_clock_manager.h"
#include "fsl_power_manager.h"
#include "fsl_mcglite_hal.h"
#include "fsl_port_hal.h"
#include "fsl_lpuart_driver.h"

#include "gpio_pins.h"
#include "SEGGER_RTT.h"
#include "warp.h"

#define WARP_FRDMKL03
#include "devSSD1331.h"
#include "devINA219.h"

#define WARP_BUILD_ENABLE_SEGGER_RTT_PRINTF

#define kWarpConstantStringI2cFailure "\rI2C failed, reg 0x%02x, code %d\n"
#define kWarpConstantStringErrorInvalidVoltage "\rInvalid supply voltage [%d] mV!"
#define kWarpConstantStringErrorSanity "\rSanity check failed!"

volatile WarpI2CDeviceState deviceINA219State;

volatile i2c_master_state_t i2cMasterState;
volatile spi_master_state_t spiMasterState;
volatile spi_master_user_config_t spiUserConfig;
volatile lpuart_user_config_t lpuartUserConfig;
volatile lpuart_state_t lpuartState;

volatile uint32_t gWarpI2cBaudRateKbps = 200;
volatile uint32_t gWarpUartBaudRateKbps = 1;
volatile uint32_t gWarpSpiBaudRateKbps = 200;
volatile uint32_t gWarpSleeptimeSeconds = 0;
volatile WarpModeMask gWarpMode = kWarpModeDisableAdcOnSleep;
volatile uint32_t gWarpI2cTimeoutMilliseconds = 5;
volatile uint32_t gWarpSpiTimeoutMicroseconds = 5;
volatile uint32_t gWarpMenuPrintDelayMilliseconds = 10;
volatile uint32_t gWarpSupplySettlingDelayMilliseconds = 1;

void sleepUntilReset(void);
void lowPowerPinStates(void);
void disableTPS82740A(void);
void disableTPS82740B(void);
void enableTPS82740A(uint16_t voltageMillivolts);
void enableTPS82740B(uint16_t voltageMillivolts);
void setTPS82740CommonControlLines(uint16_t voltageMillivolts);
void printPinDirections(void);
void dumpProcessorState(void);
void repeatRegisterReadForDeviceAndAddress(WarpSensorDevice warpSensorDevice, uint8_t baseAddress,
										   uint16_t pullupValue, bool autoIncrement, int chunkReadsPerAddress, bool chatty,
										   int spinDelay, int repetitionsPerAddress, uint16_t sssupplyMillivolts,
										   uint16_t adaptiveSssupplyMaxMillivolts, uint8_t referenceByte);
int char2int(int character);
void enableSssupply(uint16_t voltageMillivolts);
void disableSssupply(void);
void activateAllLowPowerSensorModes(bool verbose);
void powerupAllSensors(void);
uint8_t readHexByte(void);
int read4digits(void);
void printAllSensors(bool printHeadersAndCalibration, bool hexModeFlag, int menuDelayBetweenEachRun, int i2cPullupValue);

WarpStatus writeByteToI2cDeviceRegister(uint8_t i2cAddress, bool sendCommandByte, uint8_t commandByte, bool sendPayloadByte, uint8_t payloadByte);
WarpStatus writeBytesToSpi(uint8_t *payloadBytes, int payloadLength);

void warpLowPowerSecondsSleep(uint32_t sleepSeconds, bool forceAllPinsIntoLowPowerState);

/*
 *	From KSDK power_manager_demo.c <<BEGIN>>>
 */

clock_manager_error_code_t clockManagerCallbackRoutine(clock_notify_struct_t *notify, void *callbackData);

/*
 *	static clock callback table.
 */
clock_manager_callback_user_config_t clockManagerCallbackUserlevelStructure =
	{
		.callback = clockManagerCallbackRoutine,
		.callbackType = kClockManagerCallbackBeforeAfter,
		.callbackData = NULL};

static clock_manager_callback_user_config_t *clockCallbackTable[] =
	{
		&clockManagerCallbackUserlevelStructure};

clock_manager_error_code_t
clockManagerCallbackRoutine(clock_notify_struct_t *notify, void *callbackData)
{
	clock_manager_error_code_t result = kClockManagerSuccess;

	switch (notify->notifyType)
	{
	case kClockManagerNotifyBefore:
		break;
	case kClockManagerNotifyRecover:
	case kClockManagerNotifyAfter:
		break;
	default:
		result = kClockManagerError;
		break;
	}

	return result;
}

/*
 *	Override the RTC IRQ handler
 */
void RTC_IRQHandler(void)
{
	if (RTC_DRV_IsAlarmPending(0))
	{
		RTC_DRV_SetAlarmIntCmd(0, false);
	}
}

/*
 *	Override the RTC Second IRQ handler
 */
void RTC_Seconds_IRQHandler(void)
{
	gWarpSleeptimeSeconds++;
}

/*
 *	Power manager user callback
 */
power_manager_error_code_t callback0(power_manager_notify_struct_t *notify,
									 power_manager_callback_data_t *dataPtr)
{
	WarpPowerManagerCallbackStructure *callbackUserData = (WarpPowerManagerCallbackStructure *)dataPtr;
	power_manager_error_code_t status = kPowerManagerError;

	switch (notify->notifyType)
	{
	case kPowerManagerNotifyBefore:
		status = kPowerManagerSuccess;
		break;
	case kPowerManagerNotifyAfter:
		status = kPowerManagerSuccess;
		break;
	default:
		callbackUserData->errorCount++;
		break;
	}

	return status;
}

/*
 *	From KSDK power_manager_demo.c <<END>>>
 */

void sleepUntilReset(void)
{
	while (1)
	{
		warpLowPowerSecondsSleep(60, true /* forceAllPinsIntoLowPowerState */);
	}
}

void enableLPUARTpins(void)
{
	/*	Enable UART CLOCK */
	CLOCK_SYS_EnableLpuartClock(0);

	/*
	*	set UART pin association
	*	see page 99 in https://www.nxp.com/docs/en/reference-manual/KL03P24M48SF0RM.pdf
	*/

	/*
	 *	Initialize LPUART0. See KSDK13APIRM.pdf section 40.4.3, page 1353
	 *
	 */
	lpuartUserConfig.baudRate = 115;
	lpuartUserConfig.parityMode = kLpuartParityDisabled;
	lpuartUserConfig.stopBitCount = kLpuartOneStopBit;
	lpuartUserConfig.bitCountPerChar = kLpuart8BitsPerChar;

	LPUART_DRV_Init(0, (lpuart_state_t *)&lpuartState, (lpuart_user_config_t *)&lpuartUserConfig);
}

void disableLPUARTpins(void)
{
	/*
	 *	LPUART deinit
	 */
	LPUART_DRV_Deinit(0);

	/*	Warp KL03_UART_HCI_RX	--> PTB4 (GPIO)	*/
	PORT_HAL_SetMuxMode(PORTB_BASE, 4, kPortMuxAsGpio);
	/*	Warp KL03_UART_HCI_TX	--> PTB3 (GPIO) */
	PORT_HAL_SetMuxMode(PORTB_BASE, 3, kPortMuxAsGpio);

#ifdef WARP_BUILD_ENABLE_DEVPAN1326
	GPIO_DRV_ClearPinOutput(kWarpPinPAN1326_HCI_CTS);
	GPIO_DRV_ClearPinOutput(kWarpPinPAN1326_HCI_CTS);
#endif

	GPIO_DRV_ClearPinOutput(kWarpPinLPUART_HCI_TX);
	GPIO_DRV_ClearPinOutput(kWarpPinLPUART_HCI_RX);

	/* Disable LPUART CLOCK */
	CLOCK_SYS_DisableLpuartClock(0);
}

void enableSPIpins(void)
{
	CLOCK_SYS_EnableSpiClock(0);

	/*	Warp KL03_SPI_MISO	--> PTA6	(ALT3)		*/
	PORT_HAL_SetMuxMode(PORTA_BASE, 6, kPortMuxAlt3);

	/*	Warp KL03_SPI_MOSI	--> PTA7	(ALT3)		*/
	PORT_HAL_SetMuxMode(PORTA_BASE, 7, kPortMuxAlt3);

	/*	Warp KL03_SPI_SCK	--> PTB0	(ALT3)		*/
	PORT_HAL_SetMuxMode(PORTB_BASE, 0, kPortMuxAlt3);

	/*
	 *	Initialize SPI master. See KSDK13APIRM.pdf Section 70.4
	 *
	 */
	uint32_t calculatedBaudRate;
	spiUserConfig.polarity = kSpiClockPolarity_ActiveHigh;
	spiUserConfig.phase = kSpiClockPhase_FirstEdge;
	spiUserConfig.direction = kSpiMsbFirst;
	spiUserConfig.bitsPerSec = gWarpSpiBaudRateKbps * 1000;
	SPI_DRV_MasterInit(0 /* SPI master instance */, (spi_master_state_t *)&spiMasterState);
	SPI_DRV_MasterConfigureBus(0 /* SPI master instance */, (spi_master_user_config_t *)&spiUserConfig, &calculatedBaudRate);
}

void disableSPIpins(void)
{
	SPI_DRV_MasterDeinit(0);

	/*	Warp KL03_SPI_MISO	--> PTA6	(GPI)		*/
	PORT_HAL_SetMuxMode(PORTA_BASE, 6, kPortMuxAsGpio);

	/*	Warp KL03_SPI_MOSI	--> PTA7	(GPIO)		*/
	PORT_HAL_SetMuxMode(PORTA_BASE, 7, kPortMuxAsGpio);

	/*	Warp KL03_SPI_SCK	--> PTB0	(GPIO)		*/
	PORT_HAL_SetMuxMode(PORTB_BASE, 0, kPortMuxAsGpio);

	GPIO_DRV_ClearPinOutput(kWarpPinSPI_MOSI);
	GPIO_DRV_ClearPinOutput(kWarpPinSPI_MISO);
	GPIO_DRV_ClearPinOutput(kWarpPinSPI_SCK);

	CLOCK_SYS_DisableSpiClock(0);
}

void enableI2Cpins(uint16_t pullupValue)
{
	CLOCK_SYS_EnableI2cClock(0);

	/*	Warp KL03_I2C0_SCL	--> PTB3	(ALT2 == I2C)		*/
	PORT_HAL_SetMuxMode(PORTB_BASE, 3, kPortMuxAlt2);

	/*	Warp KL03_I2C0_SDA	--> PTB4	(ALT2 == I2C)		*/
	PORT_HAL_SetMuxMode(PORTB_BASE, 4, kPortMuxAlt2);

	I2C_DRV_MasterInit(0 /* I2C instance */, (i2c_master_state_t *)&i2cMasterState);
}

void disableI2Cpins(void)
{
	I2C_DRV_MasterDeinit(0 /* I2C instance */);

	/*	Warp KL03_I2C0_SCL	--> PTB3	(GPIO)			*/
	PORT_HAL_SetMuxMode(PORTB_BASE, 3, kPortMuxAsGpio);

	/*	Warp KL03_I2C0_SDA	--> PTB4	(GPIO)			*/
	PORT_HAL_SetMuxMode(PORTB_BASE, 4, kPortMuxAsGpio);

	/*
	 *	Drive the I2C pins low
	 */
	GPIO_DRV_ClearPinOutput(kWarpPinI2C0_SDA);
	GPIO_DRV_ClearPinOutput(kWarpPinI2C0_SCL);

	CLOCK_SYS_DisableI2cClock(0);
}

// TODO: add pin states for pan1326 lp states
void lowPowerPinStates(void)
{
	/*
	 *	Following Section 5 of "Power Management for Kinetis L Family" (AN5088.pdf),
	 *	we configure all pins as output and set them to a known state. We choose
	 *	to set them all to '0' since it happens that the devices we want to keep
	 *	deactivated (SI4705, PAN1326) also need '0'.
	 */

	/*
	 *			PORT A
	 */
	/*
	 *	For now, don't touch the PTA0/1/2 SWD pins. Revisit in the future.
	 */
	/*
	PORT_HAL_SetMuxMode(PORTA_BASE, 0, kPortMuxAsGpio);
	PORT_HAL_SetMuxMode(PORTA_BASE, 1, kPortMuxAsGpio);
	PORT_HAL_SetMuxMode(PORTA_BASE, 2, kPortMuxAsGpio);
	*/

	/*
	 *	PTA3 and PTA4 are the EXTAL/XTAL
	 */
	PORT_HAL_SetMuxMode(PORTA_BASE, 3, kPortPinDisabled);
	PORT_HAL_SetMuxMode(PORTA_BASE, 4, kPortPinDisabled);

	PORT_HAL_SetMuxMode(PORTA_BASE, 5, kPortMuxAsGpio);
	PORT_HAL_SetMuxMode(PORTA_BASE, 6, kPortMuxAsGpio);
	PORT_HAL_SetMuxMode(PORTA_BASE, 7, kPortMuxAsGpio);
	PORT_HAL_SetMuxMode(PORTA_BASE, 8, kPortMuxAsGpio);
	PORT_HAL_SetMuxMode(PORTA_BASE, 9, kPortMuxAsGpio);

	/*
	 *	NOTE: The KL03 has no PTA10 or PTA11
	 */

	PORT_HAL_SetMuxMode(PORTA_BASE, 12, kPortMuxAsGpio);

	/*
	 *			PORT B
	 */
	PORT_HAL_SetMuxMode(PORTB_BASE, 0, kPortMuxAsGpio);

	/*
	 *	PTB1 is connected to KL03_VDD. We have a choice of:
	 *		(1) Keep 'disabled as analog'.
	 *		(2) Set as output and drive high.
	 *
	 *	Pin state "disabled" means default functionality (ADC) is _active_
	 */
	if (gWarpMode & kWarpModeDisableAdcOnSleep)
	{
		PORT_HAL_SetMuxMode(PORTB_BASE, 1, kPortMuxAsGpio);
	}
	else
	{
		PORT_HAL_SetMuxMode(PORTB_BASE, 1, kPortPinDisabled);
	}

	PORT_HAL_SetMuxMode(PORTB_BASE, 2, kPortMuxAsGpio);

	/*
	 *	PTB3 and PTB3 (I2C pins) are true open-drain
	 *	and we purposefully leave them disabled.
	 */
	PORT_HAL_SetMuxMode(PORTB_BASE, 3, kPortPinDisabled);
	PORT_HAL_SetMuxMode(PORTB_BASE, 4, kPortPinDisabled);

	PORT_HAL_SetMuxMode(PORTB_BASE, 5, kPortMuxAsGpio);
	PORT_HAL_SetMuxMode(PORTB_BASE, 6, kPortMuxAsGpio);
	PORT_HAL_SetMuxMode(PORTB_BASE, 7, kPortMuxAsGpio);

	/*
	 *	NOTE: The KL03 has no PTB8 or PTB9
	 */

	PORT_HAL_SetMuxMode(PORTB_BASE, 10, kPortMuxAsGpio);
	PORT_HAL_SetMuxMode(PORTB_BASE, 11, kPortMuxAsGpio);

	/*
	 *	NOTE: The KL03 has no PTB12
	 */

	PORT_HAL_SetMuxMode(PORTB_BASE, 13, kPortMuxAsGpio);

	/*
	 *	Now, set all the pins (except kWarpPinKL03_VDD_ADC, the SWD pins, and the XTAL/EXTAL) to 0
	 */

	/*
	 *	If we are in mode where we disable the ADC, then drive the pin high since it is tied to KL03_VDD
	 */
	if (gWarpMode & kWarpModeDisableAdcOnSleep)
	{
		GPIO_DRV_SetPinOutput(kWarpPinKL03_VDD_ADC);
	}

	GPIO_DRV_ClearPinOutput(kWarpPinTPS82740A_CTLEN);
	GPIO_DRV_ClearPinOutput(kWarpPinTPS82740B_CTLEN);
	GPIO_DRV_ClearPinOutput(kWarpPinTPS82740_VSEL1);
	GPIO_DRV_ClearPinOutput(kWarpPinTPS82740_VSEL2);
	GPIO_DRV_ClearPinOutput(kWarpPinTPS82740_VSEL3);
	GPIO_DRV_ClearPinOutput(kWarpPinCLKOUT32K);
	GPIO_DRV_ClearPinOutput(kWarpPinTS5A3154_IN);
	GPIO_DRV_ClearPinOutput(kWarpPinSI4705_nRST);

	/*
	 *	Drive these chip selects high since they are active low:
	 */
	GPIO_DRV_SetPinOutput(kWarpPinISL23415_nCS);

	/*
	 *	When the PAN1326 is installed, note that it has the
	 *	following pull-up/down by default:
	 *
	 *		HCI_RX / kWarpPinI2C0_SCL	: pull up
	 *		HCI_TX / kWarpPinI2C0_SDA	: pull up
	 *		HCI_RTS / kWarpPinSPI_MISO	: pull up
	 *		HCI_CTS / kWarpPinSPI_MOSI	: pull up
	 *
	 *	These I/Os are 8mA (see panasonic_PAN13xx.pdf, page 10),
	 *	so we really don't want to be driving them low. We
	 *	however also have to be careful of the I2C pullup and
	 *	pull-up gating. However, driving them high leads to
	 *	higher board power dissipation even when SSSUPPLY is off
	 *	by ~80mW on board #003 (PAN1326 populated).
	 *
	 *	In revB board, with the ISL23415 DCP pullups, we also
	 *	want I2C_SCL and I2C_SDA driven high since when we
	 *	send a shutdown command to the DCP it will connect
	 *	those lines to 25570_VOUT. 
	 *
	 *	For now, we therefore leave the SPI pins low and the
	 *	I2C pins (PTB3, PTB4, which are true open-drain) disabled.
	 */

	GPIO_DRV_ClearPinOutput(kWarpPinI2C0_SDA);
	GPIO_DRV_ClearPinOutput(kWarpPinI2C0_SCL);
	GPIO_DRV_ClearPinOutput(kWarpPinSPI_MOSI);
	GPIO_DRV_ClearPinOutput(kWarpPinSPI_MISO);
	GPIO_DRV_ClearPinOutput(kWarpPinSPI_SCK);
}

void disableTPS82740A(void)
{
	GPIO_DRV_ClearPinOutput(kWarpPinTPS82740A_CTLEN);
}

void disableTPS82740B(void)
{
	GPIO_DRV_ClearPinOutput(kWarpPinTPS82740B_CTLEN);
}

void enableTPS82740A(uint16_t voltageMillivolts)
{
	setTPS82740CommonControlLines(voltageMillivolts);
	GPIO_DRV_SetPinOutput(kWarpPinTPS82740A_CTLEN);
	GPIO_DRV_ClearPinOutput(kWarpPinTPS82740B_CTLEN);

	/*
	 *	Select the TS5A3154 to use the output of the TPS82740
	 *
	 *		IN = high selects the output of the TPS82740B:
	 *		IN = low selects the output of the TPS82740A:
	 */
	GPIO_DRV_ClearPinOutput(kWarpPinTS5A3154_IN);
}

void enableTPS82740B(uint16_t voltageMillivolts)
{
	setTPS82740CommonControlLines(voltageMillivolts);
	GPIO_DRV_ClearPinOutput(kWarpPinTPS82740A_CTLEN);
	GPIO_DRV_SetPinOutput(kWarpPinTPS82740B_CTLEN);

	/*
	 *	Select the TS5A3154 to use the output of the TPS82740
	 *
	 *		IN = high selects the output of the TPS82740B:
	 *		IN = low selects the output of the TPS82740A:
	 */
	GPIO_DRV_SetPinOutput(kWarpPinTS5A3154_IN);
}

void setTPS82740CommonControlLines(uint16_t voltageMillivolts)
{
	/*
	 *	 From Manual:
	 *
	 *		TPS82740A:	VSEL1 VSEL2 VSEL3:	000-->1.8V, 111-->2.5V
	 *		TPS82740B:	VSEL1 VSEL2 VSEL3:	000-->2.6V, 111-->3.3V
	 */

	switch (voltageMillivolts)
	{
	case 2600:
	case 1800:
	{
		GPIO_DRV_ClearPinOutput(kWarpPinTPS82740_VSEL1);
		GPIO_DRV_ClearPinOutput(kWarpPinTPS82740_VSEL2);
		GPIO_DRV_ClearPinOutput(kWarpPinTPS82740_VSEL3);

		break;
	}

	case 2700:
	case 1900:
	{
		GPIO_DRV_SetPinOutput(kWarpPinTPS82740_VSEL1);
		GPIO_DRV_ClearPinOutput(kWarpPinTPS82740_VSEL2);
		GPIO_DRV_ClearPinOutput(kWarpPinTPS82740_VSEL3);

		break;
	}

	case 2800:
	case 2000:
	{
		GPIO_DRV_ClearPinOutput(kWarpPinTPS82740_VSEL1);
		GPIO_DRV_SetPinOutput(kWarpPinTPS82740_VSEL2);
		GPIO_DRV_ClearPinOutput(kWarpPinTPS82740_VSEL3);

		break;
	}

	case 2900:
	case 2100:
	{
		GPIO_DRV_SetPinOutput(kWarpPinTPS82740_VSEL1);
		GPIO_DRV_SetPinOutput(kWarpPinTPS82740_VSEL2);
		GPIO_DRV_ClearPinOutput(kWarpPinTPS82740_VSEL3);

		break;
	}

	case 3000:
	case 2200:
	{
		GPIO_DRV_ClearPinOutput(kWarpPinTPS82740_VSEL1);
		GPIO_DRV_ClearPinOutput(kWarpPinTPS82740_VSEL2);
		GPIO_DRV_SetPinOutput(kWarpPinTPS82740_VSEL3);

		break;
	}

	case 3100:
	case 2300:
	{
		GPIO_DRV_SetPinOutput(kWarpPinTPS82740_VSEL1);
		GPIO_DRV_ClearPinOutput(kWarpPinTPS82740_VSEL2);
		GPIO_DRV_SetPinOutput(kWarpPinTPS82740_VSEL3);

		break;
	}

	case 3200:
	case 2400:
	{
		GPIO_DRV_ClearPinOutput(kWarpPinTPS82740_VSEL1);
		GPIO_DRV_SetPinOutput(kWarpPinTPS82740_VSEL2);
		GPIO_DRV_SetPinOutput(kWarpPinTPS82740_VSEL3);

		break;
	}

	case 3300:
	case 2500:
	{
		GPIO_DRV_SetPinOutput(kWarpPinTPS82740_VSEL1);
		GPIO_DRV_SetPinOutput(kWarpPinTPS82740_VSEL2);
		GPIO_DRV_SetPinOutput(kWarpPinTPS82740_VSEL3);

		break;
	}

	/*
		 *	Should never happen, due to previous check in enableSssupply()
		 */
	default:
	{
#ifdef WARP_BUILD_ENABLE_SEGGER_RTT_PRINTF
		SEGGER_RTT_printf(0, RTT_CTRL_RESET RTT_CTRL_BG_BRIGHT_YELLOW RTT_CTRL_TEXT_BRIGHT_WHITE kWarpConstantStringErrorSanity RTT_CTRL_RESET "\n");
#endif
	}
	}

	/*
	 *	Vload ramp time of the TPS82740 is 800us max (datasheet, Section 8.5 / page 5)
	 */
	OSA_TimeDelay(gWarpSupplySettlingDelayMilliseconds);
}

void enableSssupply(uint16_t voltageMillivolts)
{
	if (voltageMillivolts >= 1800 && voltageMillivolts <= 2500)
	{
		enableTPS82740A(voltageMillivolts);
	}
	else if (voltageMillivolts >= 2600 && voltageMillivolts <= 3300)
	{
		enableTPS82740B(voltageMillivolts);
	}
	else
	{
#ifdef WARP_BUILD_ENABLE_SEGGER_RTT_PRINTF
		SEGGER_RTT_printf(0, RTT_CTRL_RESET RTT_CTRL_BG_BRIGHT_RED RTT_CTRL_TEXT_BRIGHT_WHITE kWarpConstantStringErrorInvalidVoltage RTT_CTRL_RESET "\n", voltageMillivolts);
#endif
	}
}

void disableSssupply(void)
{
	disableTPS82740A();
	disableTPS82740B();

	/*
	 *	Clear the pin. This sets the TS5A3154 to use the output of the TPS82740B,
	 *	which shouldn't matter in any case. The main objective here is to clear
	 *	the pin to reduce power drain.
	 *
	 *		IN = high selects the output of the TPS82740B:
	 *		IN = low selects the output of the TPS82740A:
	 */
	GPIO_DRV_SetPinOutput(kWarpPinTS5A3154_IN);

	/*
	 *	Vload ramp time of the TPS82740 is 800us max (datasheet, Section 8.5 / page 5)
	 */
	OSA_TimeDelay(gWarpSupplySettlingDelayMilliseconds);
}

void warpLowPowerSecondsSleep(uint32_t sleepSeconds, bool forceAllPinsIntoLowPowerState)
{
	/*
	 *	Set all pins into low-power states. We don't just disable all pins,
	 *	as the various devices hanging off will be left in higher power draw
	 *	state. And manuals say set pins to output to reduce power.
	 */
	if (forceAllPinsIntoLowPowerState)
	{
		lowPowerPinStates();
	}

	warpSetLowPowerMode(kWarpPowerModeVLPR, 0);
	warpSetLowPowerMode(kWarpPowerModeVLPS, sleepSeconds);
}

int main(void)
{
	uint8_t key;
	WarpSensorDevice menuTargetSensor = kWarpSensorBMX055accel;
	volatile WarpI2CDeviceState *menuI2cDevice = NULL;
	uint16_t menuI2cPullupValue = 32768;
	uint8_t menuRegisterAddress = 0x00;
	uint16_t menuSupplyVoltage = 0;

	rtc_datetime_t warpBootDate;

	power_manager_user_config_t warpPowerModeWaitConfig;
	power_manager_user_config_t warpPowerModeStopConfig;
	power_manager_user_config_t warpPowerModeVlpwConfig;
	power_manager_user_config_t warpPowerModeVlpsConfig;
	power_manager_user_config_t warpPowerModeVlls0Config;
	power_manager_user_config_t warpPowerModeVlls1Config;
	power_manager_user_config_t warpPowerModeVlls3Config;
	power_manager_user_config_t warpPowerModeRunConfig;

	const power_manager_user_config_t warpPowerModeVlprConfig = {
		.mode = kPowerManagerVlpr,
		.sleepOnExitValue = false,
		.sleepOnExitOption = false};

	power_manager_user_config_t const *powerConfigs[] = {
		/*
							 *	NOTE: This order is depended on by POWER_SYS_SetMode()
							 *
							 *	See KSDK13APIRM.pdf Section 55.5.3
							 */
		&warpPowerModeWaitConfig,
		&warpPowerModeStopConfig,
		&warpPowerModeVlprConfig,
		&warpPowerModeVlpwConfig,
		&warpPowerModeVlpsConfig,
		&warpPowerModeVlls0Config,
		&warpPowerModeVlls1Config,
		&warpPowerModeVlls3Config,
		&warpPowerModeRunConfig,
	};

	WarpPowerManagerCallbackStructure powerManagerCallbackStructure;

	/*
	 *	Callback configuration structure for power manager
	 */
	const power_manager_callback_user_config_t callbackCfg0 = {
		callback0,
		kPowerManagerCallbackBeforeAfter,
		(power_manager_callback_data_t *)&powerManagerCallbackStructure};

	/*
	 *	Pointers to power manager callbacks.
	 */
	power_manager_callback_user_config_t const *callbacks[] = {
		&callbackCfg0};

	/*
	 *	Enable clock for I/O PORT A and PORT B
	 */
	CLOCK_SYS_EnablePortClock(0);
	CLOCK_SYS_EnablePortClock(1);

	/*
	 *	Setup board clock source.
	 */
	g_xtal0ClkFreq = 32768U;

	/*
	 *	Initialize KSDK Operating System Abstraction layer (OSA) layer.
	 */
	OSA_Init();

	/*
	 *	Setup SEGGER RTT to output as much as fits in buffers.
	 *
	 *	Using SEGGER_RTT_MODE_BLOCK_IF_FIFO_FULL can lead to deadlock, since
	 *	we might have SWD disabled at time of blockage.
	 */
	SEGGER_RTT_ConfigUpBuffer(0, NULL, NULL, 0, SEGGER_RTT_MODE_NO_BLOCK_TRIM);

	SEGGER_RTT_WriteString(0, "\n\n\n\rBooting Warp, in 3... ");
	OSA_TimeDelay(200);
	SEGGER_RTT_WriteString(0, "2... ");
	OSA_TimeDelay(200);
	SEGGER_RTT_WriteString(0, "1...\n\r");
	OSA_TimeDelay(200);

	/*
	 *	Configure Clock Manager to default, and set callback for Clock Manager mode transition.
	 *
	 *	See "Clocks and Low Power modes with KSDK and Processor Expert" document (Low_Power_KSDK_PEx.pdf)
	 */
	CLOCK_SYS_Init(g_defaultClockConfigurations,
				   CLOCK_CONFIG_NUM,
				   &clockCallbackTable,
				   ARRAY_SIZE(clockCallbackTable));
	CLOCK_SYS_UpdateConfiguration(CLOCK_CONFIG_INDEX_FOR_RUN, kClockManagerPolicyForcible);

	/*
	 *	Initialize RTC Driver
	 */
	RTC_DRV_Init(0);

	/*
	 *	Set initial date to 1st January 2016 00:00, and set date via RTC driver
	 */
	warpBootDate.year = 2016U;
	warpBootDate.month = 1U;
	warpBootDate.day = 1U;
	warpBootDate.hour = 0U;
	warpBootDate.minute = 0U;
	warpBootDate.second = 0U;
	RTC_DRV_SetDatetime(0, &warpBootDate);

	/*
	 *	Setup Power Manager Driver
	 */
	memset(&powerManagerCallbackStructure, 0, sizeof(WarpPowerManagerCallbackStructure));

	warpPowerModeVlpwConfig = warpPowerModeVlprConfig;
	warpPowerModeVlpwConfig.mode = kPowerManagerVlpw;

	warpPowerModeVlpsConfig = warpPowerModeVlprConfig;
	warpPowerModeVlpsConfig.mode = kPowerManagerVlps;

	warpPowerModeWaitConfig = warpPowerModeVlprConfig;
	warpPowerModeWaitConfig.mode = kPowerManagerWait;

	warpPowerModeStopConfig = warpPowerModeVlprConfig;
	warpPowerModeStopConfig.mode = kPowerManagerStop;

	warpPowerModeVlls0Config = warpPowerModeVlprConfig;
	warpPowerModeVlls0Config.mode = kPowerManagerVlls0;

	warpPowerModeVlls1Config = warpPowerModeVlprConfig;
	warpPowerModeVlls1Config.mode = kPowerManagerVlls1;

	warpPowerModeVlls3Config = warpPowerModeVlprConfig;
	warpPowerModeVlls3Config.mode = kPowerManagerVlls3;

	warpPowerModeRunConfig.mode = kPowerManagerRun;

	POWER_SYS_Init(&powerConfigs,
				   sizeof(powerConfigs) / sizeof(power_manager_user_config_t *),
				   &callbacks,
				   sizeof(callbacks) / sizeof(power_manager_callback_user_config_t *));

	/*
	 *	Switch CPU to Very Low Power Run (VLPR) mode
	 */
	warpSetLowPowerMode(kWarpPowerModeVLPR, 0);

	/*
	 *	Initialize the GPIO pins with the appropriate pull-up, etc.,
	 *	defined in the inputPins and outputPins arrays (gpio_pins.c).
	 *
	 *	See also Section 30.3.3 GPIO Initialization of KSDK13APIRM.pdf
	 */
	GPIO_DRV_Init(inputPins /* input pins */, outputPins /* output pins */);

	/*
	 *	Note that it is lowPowerPinStates() that sets the pin mux mode,
	 *	so until we call it pins are in their default state.
	 */
	lowPowerPinStates();

	/*
	 *	Toggle LED3 (kWarpPinSI4705_nRST)
	 */
	GPIO_DRV_SetPinOutput(kWarpPinSI4705_nRST);
	OSA_TimeDelay(200);
	GPIO_DRV_ClearPinOutput(kWarpPinSI4705_nRST);
	OSA_TimeDelay(200);
	GPIO_DRV_SetPinOutput(kWarpPinSI4705_nRST);
	OSA_TimeDelay(200);
	GPIO_DRV_ClearPinOutput(kWarpPinSI4705_nRST);
	OSA_TimeDelay(200);
	GPIO_DRV_SetPinOutput(kWarpPinSI4705_nRST);
	OSA_TimeDelay(200);
	GPIO_DRV_ClearPinOutput(kWarpPinSI4705_nRST);

	/*
	 *	Initialize all the sensors
	 */
	initINA219(0x40 /* i2cAddress */, &deviceINA219State);

	/*
	 *	Make sure SCALED_SENSOR_SUPPLY is off.
	 *
	 *	(There's no point in calling activateAllLowPowerSensorModes())
	 */
	disableSssupply();

	/*
	 *	TODO: initialize the kWarpPinKL03_VDD_ADC, write routines to read the VDD and temperature
	 */

	devSSD1331init();
	while (1)
	{
		/*
		 *	Do not, e.g., lowPowerPinStates() on each iteration, because we actually
		 *	want to use menu to progressiveley change the machine state with various
		 *	commands.
		 */

		/*
		 *	We break up the prints with small delays to allow us to use small RTT print
		 *	buffers without overrunning them when at max CPU speed.
		 */
		// SEGGER_RTT_WriteString(0, "Test");
		// OSA_TimeDelay(gWarpMenuPrintDelayMilliseconds);
		// SEGGER_RTT_WriteString(0, "\r[  \t\t\t\t      Cambridge / Physcomplab   \t\t\t\t  ]\n\n");
		// OSA_TimeDelay(gWarpMenuPrintDelayMilliseconds);
		// SEGGER_RTT_printf(0, "\r\tSupply=%dmV,\tDefault Target Read Register=0x%02x\n",
		// 				  menuSupplyVoltage, menuRegisterAddress);
		// OSA_TimeDelay(gWarpMenuPrintDelayMilliseconds);
	}

	return 0;
}

void loopForSensorWithNumberOfBytes(const char *tagString,
									WarpStatus (*readSensorRegisterFunction)(uint8_t deviceRegister, int numberOfBytes),
									volatile WarpI2CDeviceState *i2cDeviceState,
									volatile WarpSPIDeviceState *spiDeviceState,
									uint8_t baseAddress,
									uint8_t minAddress,
									uint8_t maxAddress,
									int repetitionsPerAddress,
									int chunkReadsPerAddress,
									int spinDelay,
									bool autoIncrement,
									uint16_t sssupplyMillivolts,
									uint8_t referenceByte,
									uint16_t adaptiveSssupplyMaxMillivolts,
									bool chatty,
									int numberOfBytes)
{
	WarpStatus status;
	uint8_t address = baseAddress;
	int readCount = repetitionsPerAddress + 1;
	int nSuccesses = 0;
	int nFailures = 0;
	int nCorrects = 0;
	int nBadCommands = 0;
	uint16_t actualSssupplyMillivolts = sssupplyMillivolts;

	if ((!spiDeviceState && !i2cDeviceState) ||
		(spiDeviceState && i2cDeviceState))
	{
#ifdef WARP_BUILD_ENABLE_SEGGER_RTT_PRINTF
		SEGGER_RTT_printf(0, RTT_CTRL_RESET RTT_CTRL_BG_BRIGHT_YELLOW RTT_CTRL_TEXT_BRIGHT_WHITE kWarpConstantStringErrorSanity RTT_CTRL_RESET "\n");
#endif
	}

	enableSssupply(actualSssupplyMillivolts);
	SEGGER_RTT_WriteString(0, tagString);

	/*
	 *	Keep on repeating until we are above the maxAddress, or just once if not autoIncrement-ing
	 *	This is checked for at the tail end of the loop.
	 */
	while (true)
	{
		for (int i = 0; i < readCount; i++)
			for (int j = 0; j < chunkReadsPerAddress; j++)
			{
				status = readSensorRegisterFunction(address + j, numberOfBytes /* numberOfBytes */);
				if (status == kWarpStatusOK)
				{
					nSuccesses++;
					if (actualSssupplyMillivolts > sssupplyMillivolts)
					{
						actualSssupplyMillivolts -= 100;
						enableSssupply(actualSssupplyMillivolts);
					}

					if (spiDeviceState)
					{
						if (referenceByte == spiDeviceState->spiSinkBuffer[2])
						{
							nCorrects++;
						}

						if (chatty)
						{
#ifdef WARP_BUILD_ENABLE_SEGGER_RTT_PRINTF
							SEGGER_RTT_printf(0, "\r\t0x%02x --> [0x%02x 0x%02x 0x%02x]\n",
											  address + j,
											  spiDeviceState->spiSinkBuffer[0],
											  spiDeviceState->spiSinkBuffer[1],
											  spiDeviceState->spiSinkBuffer[2]);
#endif
						}
					}
					else
					{
						if (referenceByte == i2cDeviceState->i2cBuffer[0])
						{
							nCorrects++;
						}

						if (chatty)
						{
#ifdef WARP_BUILD_ENABLE_SEGGER_RTT_PRINTF
							SEGGER_RTT_printf(0, "\r\t0x%02x --> ", address + j);
							for (int k = 0; k < numberOfBytes; k++)
							{
								SEGGER_RTT_printf(0, "0x%02x ", i2cDeviceState->i2cBuffer[k]);
							}
							SEGGER_RTT_printf(0, "\n");
#endif
						}
					}
				}
				else if (status == kWarpStatusDeviceCommunicationFailed)
				{
#ifdef WARP_BUILD_ENABLE_SEGGER_RTT_PRINTF
					SEGGER_RTT_printf(0, "\r\t0x%02x --> ----\n",
									  address + j);
#endif

					nFailures++;
					if (actualSssupplyMillivolts < adaptiveSssupplyMaxMillivolts)
					{
						actualSssupplyMillivolts += 100;
						enableSssupply(actualSssupplyMillivolts);
					}
				}
				else if (status == kWarpStatusBadDeviceCommand)
				{
					nBadCommands++;
				}

				if (spinDelay > 0)
				{
					OSA_TimeDelay(spinDelay);
				}
			}

		if (autoIncrement)
		{
			address++;
		}

		if (address > maxAddress || !autoIncrement)
		{
			/*
			 *	We either iterated over all possible addresses, or were asked to do only 
			 *	one address anyway (i.e. don't increment), so we're done.
			 */
			break;
		}
	}

	/*
	 *	We intersperse RTT_printfs with forced delays to allow us to use small
	 *	print buffers even in RUN mode.
	 */
#ifdef WARP_BUILD_ENABLE_SEGGER_RTT_PRINTF
	SEGGER_RTT_printf(0, "\r\n\t%d/%d success rate.\n", nSuccesses, (nSuccesses + nFailures));
	OSA_TimeDelay(50);
	SEGGER_RTT_printf(0, "\r\t%d/%d successes matched ref. value of 0x%02x.\n", nCorrects, nSuccesses, referenceByte);
	OSA_TimeDelay(50);
	SEGGER_RTT_printf(0, "\r\t%d bad commands.\n\n", nBadCommands);
	OSA_TimeDelay(50);
#endif

	return;
}

void loopForSensor(const char *tagString,
				   WarpStatus (*readSensorRegisterFunction)(uint8_t deviceRegister, int numberOfBytes),
				   volatile WarpI2CDeviceState *i2cDeviceState,
				   volatile WarpSPIDeviceState *spiDeviceState,
				   uint8_t baseAddress,
				   uint8_t minAddress,
				   uint8_t maxAddress,
				   int repetitionsPerAddress,
				   int chunkReadsPerAddress,
				   int spinDelay,
				   bool autoIncrement,
				   uint16_t sssupplyMillivolts,
				   uint8_t referenceByte,
				   uint16_t adaptiveSssupplyMaxMillivolts,
				   bool chatty)
{
	return loopForSensorWithNumberOfBytes(tagString,
										  readSensorRegisterFunction,
										  i2cDeviceState,
										  spiDeviceState,
										  baseAddress,
										  minAddress,
										  maxAddress,
										  repetitionsPerAddress,
										  chunkReadsPerAddress,
										  spinDelay,
										  autoIncrement,
										  sssupplyMillivolts,
										  referenceByte,
										  adaptiveSssupplyMaxMillivolts,
										  chatty,
										  1);
}

void repeatRegisterReadForDeviceAndAddress(WarpSensorDevice warpSensorDevice, uint8_t baseAddress, uint16_t pullupValue, bool autoIncrement, int chunkReadsPerAddress, bool chatty, int spinDelay, int repetitionsPerAddress, uint16_t sssupplyMillivolts, uint16_t adaptiveSssupplyMaxMillivolts, uint8_t referenceByte)
{
	if (warpSensorDevice != kWarpSensorADXL362)
	{
		enableI2Cpins(pullupValue);
	}

	switch (warpSensorDevice)
	{
	case kWarpSensorADXL362:
	{
		/*
			 *	ADXL362: VDD 1.6--3.5
			 */
#ifdef WARP_BUILD_ENABLE_DEVADXL362
		loopForSensor("\r\nADXL362:\n\r",			 /*	tagString			*/
					  &readSensorRegisterADXL362,	/*	readSensorRegisterFunction	*/
					  NULL,							 /*	i2cDeviceState			*/
					  &deviceADXL362State,			 /*	spiDeviceState			*/
					  baseAddress,					 /*	baseAddress			*/
					  0x00,							 /*	minAddress			*/
					  0x2E,							 /*	maxAddress			*/
					  repetitionsPerAddress,		 /*	repetitionsPerAddress		*/
					  chunkReadsPerAddress,			 /*	chunkReadsPerAddress		*/
					  spinDelay,					 /*	spinDelay			*/
					  autoIncrement,				 /*	autoIncrement			*/
					  sssupplyMillivolts,			 /*	sssupplyMillivolts		*/
					  referenceByte,				 /*	referenceByte			*/
					  adaptiveSssupplyMaxMillivolts, /*	adaptiveSssupplyMaxMillivolts	*/
					  chatty						 /*	chatty				*/
		);
#else
		SEGGER_RTT_WriteString(0, "\r\n\tADXL362 Read Aborted. Device Disabled :(");
#endif
		break;
	}

	case kWarpSensorMMA8451Q:
	{
		/*
			 *	MMA8451Q: VDD 1.95--3.6
			 */
#ifdef WARP_BUILD_ENABLE_DEVMMA8451Q
		loopForSensor("\r\nMMA8451Q:\n\r",			 /*	tagString			*/
					  &readSensorRegisterMMA8451Q,   /*	readSensorRegisterFunction	*/
					  &deviceMMA8451QState,			 /*	i2cDeviceState			*/
					  NULL,							 /*	spiDeviceState			*/
					  baseAddress,					 /*	baseAddress			*/
					  0x00,							 /*	minAddress			*/
					  0x31,							 /*	maxAddress			*/
					  repetitionsPerAddress,		 /*	repetitionsPerAddress		*/
					  chunkReadsPerAddress,			 /*	chunkReadsPerAddress		*/
					  spinDelay,					 /*	spinDelay			*/
					  autoIncrement,				 /*	autoIncrement			*/
					  sssupplyMillivolts,			 /*	sssupplyMillivolts		*/
					  referenceByte,				 /*	referenceByte			*/
					  adaptiveSssupplyMaxMillivolts, /*	adaptiveSssupplyMaxMillivolts	*/
					  chatty						 /*	chatty				*/
		);
#else
		SEGGER_RTT_WriteString(0, "\r\n\tMMA8451Q Read Aborted. Device Disabled :(");
#endif
		break;
	}

	case kWarpSensorBME680:
	{
		/*
			 *	BME680: VDD 1.7--3.6
			 */
#ifdef WARP_BUILD_ENABLE_DEVBME680
		loopForSensor("\r\nBME680:\n\r",			 /*	tagString			*/
					  &readSensorRegisterBME680,	 /*	readSensorRegisterFunction	*/
					  &deviceBME680State,			 /*	i2cDeviceState			*/
					  NULL,							 /*	spiDeviceState			*/
					  baseAddress,					 /*	baseAddress			*/
					  0x1D,							 /*	minAddress			*/
					  0x75,							 /*	maxAddress			*/
					  repetitionsPerAddress,		 /*	repetitionsPerAddress		*/
					  chunkReadsPerAddress,			 /*	chunkReadsPerAddress		*/
					  spinDelay,					 /*	spinDelay			*/
					  autoIncrement,				 /*	autoIncrement			*/
					  sssupplyMillivolts,			 /*	sssupplyMillivolts		*/
					  referenceByte,				 /*	referenceByte			*/
					  adaptiveSssupplyMaxMillivolts, /*	adaptiveSssupplyMaxMillivolts	*/
					  chatty						 /*	chatty				*/
		);
#else
		SEGGER_RTT_WriteString(0, "\r\n\nBME680 Read Aborted. Device Disabled :(");
#endif
		break;
	}

	case kWarpSensorBMX055accel:
	{
		/*
			 *	BMX055accel: VDD 2.4V -- 3.6V
			 */
#ifdef WARP_BUILD_ENABLE_DEVBMX055
		loopForSensor("\r\nBMX055accel:\n\r",		  /*	tagString			*/
					  &readSensorRegisterBMX055accel, /*	readSensorRegisterFunction	*/
					  &deviceBMX055accelState,		  /*	i2cDeviceState			*/
					  NULL,							  /*	spiDeviceState			*/
					  baseAddress,					  /*	baseAddress			*/
					  0x00,							  /*	minAddress			*/
					  0x39,							  /*	maxAddress			*/
					  repetitionsPerAddress,		  /*	repetitionsPerAddress		*/
					  chunkReadsPerAddress,			  /*	chunkReadsPerAddress		*/
					  spinDelay,					  /*	spinDelay			*/
					  autoIncrement,				  /*	autoIncrement			*/
					  sssupplyMillivolts,			  /*	sssupplyMillivolts		*/
					  referenceByte,				  /*	referenceByte			*/
					  adaptiveSssupplyMaxMillivolts,  /*	adaptiveSssupplyMaxMillivolts	*/
					  chatty						  /*	chatty				*/
		);
#else
		SEGGER_RTT_WriteString(0, "\r\n\tBMX055accel Read Aborted. Device Disabled :( ");
#endif
		break;
	}

	case kWarpSensorBMX055gyro:
	{
		/*
			 *	BMX055gyro: VDD 2.4V -- 3.6V
			 */
#ifdef WARP_BUILD_ENABLE_DEVBMX055
		loopForSensor("\r\nBMX055gyro:\n\r",		 /*	tagString			*/
					  &readSensorRegisterBMX055gyro, /*	readSensorRegisterFunction	*/
					  &deviceBMX055gyroState,		 /*	i2cDeviceState			*/
					  NULL,							 /*	spiDeviceState			*/
					  baseAddress,					 /*	baseAddress			*/
					  0x00,							 /*	minAddress			*/
					  0x39,							 /*	maxAddress			*/
					  repetitionsPerAddress,		 /*	repetitionsPerAddress		*/
					  chunkReadsPerAddress,			 /*	chunkReadsPerAddress		*/
					  spinDelay,					 /*	spinDelay			*/
					  autoIncrement,				 /*	autoIncrement			*/
					  sssupplyMillivolts,			 /*	sssupplyMillivolts		*/
					  referenceByte,				 /*	referenceByte			*/
					  adaptiveSssupplyMaxMillivolts, /*	adaptiveSssupplyMaxMillivolts	*/
					  chatty						 /*	chatty				*/
		);
#else
		SEGGER_RTT_WriteString(0, "\r\n\tBMX055gyro Read Aborted. Device Disabled :( ");
#endif
		break;
	}

	case kWarpSensorBMX055mag:
	{
		/*
			 *	BMX055mag: VDD 2.4V -- 3.6V
			 */
#ifdef WARP_BUILD_ENABLE_DEVBMX055
		loopForSensor("\r\nBMX055mag:\n\r",			 /*	tagString			*/
					  &readSensorRegisterBMX055mag,  /*	readSensorRegisterFunction	*/
					  &deviceBMX055magState,		 /*	i2cDeviceState			*/
					  NULL,							 /*	spiDeviceState			*/
					  baseAddress,					 /*	baseAddress			*/
					  0x40,							 /*	minAddress			*/
					  0x52,							 /*	maxAddress			*/
					  repetitionsPerAddress,		 /*	repetitionsPerAddress		*/
					  chunkReadsPerAddress,			 /*	chunkReadsPerAddress		*/
					  spinDelay,					 /*	spinDelay			*/
					  autoIncrement,				 /*	autoIncrement			*/
					  sssupplyMillivolts,			 /*	sssupplyMillivolts		*/
					  referenceByte,				 /*	referenceByte			*/
					  adaptiveSssupplyMaxMillivolts, /*	adaptiveSssupplyMaxMillivolts	*/
					  chatty						 /*	chatty				*/
		);
#else
		SEGGER_RTT_WriteString(0, "\r\n\t BMX055mag Read Aborted. Device Disabled :( ");
#endif
		break;
	}

	case kWarpSensorMAG3110:
	{
		/*
			 *	MAG3110: VDD 1.95 -- 3.6
			 */
#ifdef WARP_BUILD_ENABLE_DEVMAG3110
		loopForSensor("\r\nMAG3110:\n\r",			 /*	tagString			*/
					  &readSensorRegisterMAG3110,	/*	readSensorRegisterFunction	*/
					  &deviceMAG3110State,			 /*	i2cDeviceState			*/
					  NULL,							 /*	spiDeviceState			*/
					  baseAddress,					 /*	baseAddress			*/
					  0x00,							 /*	minAddress			*/
					  0x11,							 /*	maxAddress			*/
					  repetitionsPerAddress,		 /*	repetitionsPerAddress		*/
					  chunkReadsPerAddress,			 /*	chunkReadsPerAddress		*/
					  spinDelay,					 /*	spinDelay			*/
					  autoIncrement,				 /*	autoIncrement			*/
					  sssupplyMillivolts,			 /*	sssupplyMillivolts		*/
					  referenceByte,				 /*	referenceByte			*/
					  adaptiveSssupplyMaxMillivolts, /*	adaptiveSssupplyMaxMillivolts	*/
					  chatty						 /*	chatty				*/
		);
#else
		SEGGER_RTT_WriteString(0, "\r\n\tMAG3110 Read Aborted. Device Disabled :( ");
#endif
		break;
	}

	case kWarpSensorL3GD20H:
	{
		/*
			 *	L3GD20H: VDD 2.2V -- 3.6V
			 */
#ifdef WARP_BUILD_ENABLE_DEVL3GD20H
		loopForSensor("\r\nL3GD20H:\n\r",			 /*	tagString			*/
					  &readSensorRegisterL3GD20H,	/*	readSensorRegisterFunction	*/
					  &deviceL3GD20HState,			 /*	i2cDeviceState			*/
					  NULL,							 /*	spiDeviceState			*/
					  baseAddress,					 /*	baseAddress			*/
					  0x0F,							 /*	minAddress			*/
					  0x39,							 /*	maxAddress			*/
					  repetitionsPerAddress,		 /*	repetitionsPerAddress		*/
					  chunkReadsPerAddress,			 /*	chunkReadsPerAddress		*/
					  spinDelay,					 /*	spinDelay			*/
					  autoIncrement,				 /*	autoIncrement			*/
					  sssupplyMillivolts,			 /*	sssupplyMillivolts		*/
					  referenceByte,				 /*	referenceByte			*/
					  adaptiveSssupplyMaxMillivolts, /*	adaptiveSssupplyMaxMillivolts	*/
					  chatty						 /*	chatty				*/
		);
#else
		SEGGER_RTT_WriteString(0, "\r\n\tL3GD20H Read Aborted. Device Disabled :( ");
#endif
		break;
	}

	case kWarpSensorLPS25H:
	{
		/*
			 *	LPS25H: VDD 1.7V -- 3.6V
			 */
#ifdef WARP_BUILD_ENABLE_DEVLPS25H
		loopForSensor("\r\nLPS25H:\n\r",			 /*	tagString			*/
					  &readSensorRegisterLPS25H,	 /*	readSensorRegisterFunction	*/
					  &deviceLPS25HState,			 /*	i2cDeviceState			*/
					  NULL,							 /*	spiDeviceState			*/
					  baseAddress,					 /*	baseAddress			*/
					  0x08,							 /*	minAddress			*/
					  0x24,							 /*	maxAddress			*/
					  repetitionsPerAddress,		 /*	repetitionsPerAddress		*/
					  chunkReadsPerAddress,			 /*	chunkReadsPerAddress		*/
					  spinDelay,					 /*	spinDelay			*/
					  autoIncrement,				 /*	autoIncrement			*/
					  sssupplyMillivolts,			 /*	sssupplyMillivolts		*/
					  referenceByte,				 /*	referenceByte			*/
					  adaptiveSssupplyMaxMillivolts, /*	adaptiveSssupplyMaxMillivolts	*/
					  chatty						 /*	chatty				*/
		);
#else
		SEGGER_RTT_WriteString(0, "\r\n\tLPS25H Read Aborted. Device Disabled :( ");
#endif
		break;
	}

	case kWarpSensorTCS34725:
	{
		/*
			 *	TCS34725: VDD 2.7V -- 3.3V
			 */
#ifdef WARP_BUILD_ENABLE_DEVTCS34725
		loopForSensor("\r\nTCS34725:\n\r",			 /*	tagString			*/
					  &readSensorRegisterTCS34725,   /*	readSensorRegisterFunction	*/
					  &deviceTCS34725State,			 /*	i2cDeviceState			*/
					  NULL,							 /*	spiDeviceState			*/
					  baseAddress,					 /*	baseAddress			*/
					  0x00,							 /*	minAddress			*/
					  0x1D,							 /*	maxAddress			*/
					  repetitionsPerAddress,		 /*	repetitionsPerAddress		*/
					  chunkReadsPerAddress,			 /*	chunkReadsPerAddress		*/
					  spinDelay,					 /*	spinDelay			*/
					  autoIncrement,				 /*	autoIncrement			*/
					  sssupplyMillivolts,			 /*	sssupplyMillivolts		*/
					  referenceByte,				 /*	referenceByte			*/
					  adaptiveSssupplyMaxMillivolts, /*	adaptiveSssupplyMaxMillivolts	*/
					  chatty						 /*	chatty				*/
		);
#else
		SEGGER_RTT_WriteString(0, "\r\n\tTCS34725 Read Aborted. Device Disabled :( ");
#endif
		break;
	}

	case kWarpSensorSI4705:
	{
		/*
			 *	SI4705: VDD 2.7V -- 5.5V
			 */
#ifdef WARP_BUILD_ENABLE_DEVSI4705
		loopForSensor("\r\nSI4705:\n\r",			 /*	tagString			*/
					  &readSensorRegisterSI4705,	 /*	readSensorRegisterFunction	*/
					  &deviceSI4705State,			 /*	i2cDeviceState			*/
					  NULL,							 /*	spiDeviceState			*/
					  baseAddress,					 /*	baseAddress			*/
					  0x00,							 /*	minAddress			*/
					  0x09,							 /*	maxAddress			*/
					  repetitionsPerAddress,		 /*	repetitionsPerAddress		*/
					  chunkReadsPerAddress,			 /*	chunkReadsPerAddress		*/
					  spinDelay,					 /*	spinDelay			*/
					  autoIncrement,				 /*	autoIncrement			*/
					  sssupplyMillivolts,			 /*	sssupplyMillivolts		*/
					  referenceByte,				 /*	referenceByte			*/
					  adaptiveSssupplyMaxMillivolts, /*	adaptiveSssupplyMaxMillivolts	*/
					  chatty						 /*	chatty				*/
		);
#else
		SEGGER_RTT_WriteString(0, "\r\n\tSI4705 Read Aborted. Device Disabled :( ");
#endif
		break;
	}

	case kWarpSensorHDC1000:
	{
		/*
			 *	HDC1000: VDD 3V--5V
			 */
#ifdef WARP_BUILD_ENABLE_DEVHDC1000
		loopForSensor("\r\nHDC1000:\n\r",			 /*	tagString			*/
					  &readSensorRegisterHDC1000,	/*	readSensorRegisterFunction	*/
					  &deviceHDC1000State,			 /*	i2cDeviceState			*/
					  NULL,							 /*	spiDeviceState			*/
					  baseAddress,					 /*	baseAddress			*/
					  0x00,							 /*	minAddress			*/
					  0x1F,							 /*	maxAddress			*/
					  repetitionsPerAddress,		 /*	repetitionsPerAddress		*/
					  chunkReadsPerAddress,			 /*	chunkReadsPerAddress		*/
					  spinDelay,					 /*	spinDelay			*/
					  autoIncrement,				 /*	autoIncrement			*/
					  sssupplyMillivolts,			 /*	sssupplyMillivolts		*/
					  referenceByte,				 /*	referenceByte			*/
					  adaptiveSssupplyMaxMillivolts, /*	adaptiveSssupplyMaxMillivolts	*/
					  chatty						 /*	chatty				*/
		);
#else
		SEGGER_RTT_WriteString(0, "\r\n\tHDC1000 Read Aborted. Device Disabled :( ");
#endif
		break;
	}

	case kWarpSensorSI7021:
	{
		/*
			 *	SI7021: VDD 1.9V -- 3.6V
			 */
#ifdef WARP_BUILD_ENABLE_DEVSI7021
		loopForSensor("\r\nSI7021:\n\r",			 /*	tagString			*/
					  &readSensorRegisterSI7021,	 /*	readSensorRegisterFunction	*/
					  &deviceSI7021State,			 /*	i2cDeviceState			*/
					  NULL,							 /*	spiDeviceState			*/
					  baseAddress,					 /*	baseAddress			*/
					  0x00,							 /*	minAddress			*/
					  0x09,							 /*	maxAddress			*/
					  repetitionsPerAddress,		 /*	repetitionsPerAddress		*/
					  chunkReadsPerAddress,			 /*	chunkReadsPerAddress		*/
					  spinDelay,					 /*	spinDelay			*/
					  autoIncrement,				 /*	autoIncrement			*/
					  sssupplyMillivolts,			 /*	sssupplyMillivolts		*/
					  referenceByte,				 /*	referenceByte			*/
					  adaptiveSssupplyMaxMillivolts, /*	adaptiveSssupplyMaxMillivolts	*/
					  chatty						 /*	chatty				*/
		);
#else
		SEGGER_RTT_WriteString(0, "\r\n\tSI7021 Read Aborted. Device Disabled :( ");
#endif
		break;
	}

	case kWarpSensorCCS811:
	{
		/*
			 *	CCS811: VDD 1.8V -- 3.6V
			 */
#ifdef WARP_BUILD_ENABLE_DEVCCS811
		loopForSensor("\r\nCCS811:\n\r",			 /*	tagString			*/
					  &readSensorRegisterCCS811,	 /*	readSensorRegisterFunction	*/
					  &deviceCCS811State,			 /*	i2cDeviceState			*/
					  NULL,							 /*	spiDeviceState			*/
					  baseAddress,					 /*	baseAddress			*/
					  0x00,							 /*	minAddress			*/
					  0xFF,							 /*	maxAddress			*/
					  repetitionsPerAddress,		 /*	repetitionsPerAddress		*/
					  chunkReadsPerAddress,			 /*	chunkReadsPerAddress		*/
					  spinDelay,					 /*	spinDelay			*/
					  autoIncrement,				 /*	autoIncrement			*/
					  sssupplyMillivolts,			 /*	sssupplyMillivolts		*/
					  referenceByte,				 /*	referenceByte			*/
					  adaptiveSssupplyMaxMillivolts, /*	adaptiveSssupplyMaxMillivolts	*/
					  chatty						 /*	chatty				*/
		);
#else
		SEGGER_RTT_WriteString(0, "\r\n\tCCS811 Read Aborted. Device Disabled :( ");
#endif
		break;
	}

	case kWarpSensorAMG8834:
	{
		/*
			 *	AMG8834: VDD ?V -- ?V
			 */
#ifdef WARP_BUILD_ENABLE_DEVAMG8834
		loopForSensor("\r\nAMG8834:\n\r",			 /*	tagString			*/
					  &readSensorRegisterAMG8834,	/*	readSensorRegisterFunction	*/
					  &deviceAMG8834State,			 /*	i2cDeviceState			*/
					  NULL,							 /*	spiDeviceState			*/
					  baseAddress,					 /*	baseAddress			*/
					  0x00,							 /*	minAddress			*/
					  0xFF,							 /*	maxAddress			*/
					  repetitionsPerAddress,		 /*	repetitionsPerAddress		*/
					  chunkReadsPerAddress,			 /*	chunkReadsPerAddress		*/
					  spinDelay,					 /*	spinDelay			*/
					  autoIncrement,				 /*	autoIncrement			*/
					  sssupplyMillivolts,			 /*	sssupplyMillivolts		*/
					  referenceByte,				 /*	referenceByte			*/
					  adaptiveSssupplyMaxMillivolts, /*	adaptiveSssupplyMaxMillivolts	*/
					  chatty						 /*	chatty				*/
		);
#else
		SEGGER_RTT_WriteString(0, "\r\n\tAMG8834 Read Aborted. Device Disabled :( ");
#endif
		break;
	}

	case kWarpSensorAS7262:
	{
		/*
			 *	AS7262: VDD 2.7--3.6
			 */
#ifdef WARP_BUILD_ENABLE_DEVAS7262
		loopForSensor("\r\nAS7262:\n\r",			 /*	tagString			*/
					  &readSensorRegisterAS7262,	 /*	readSensorRegisterFunction	*/
					  &deviceAS7262State,			 /*	i2cDeviceState			*/
					  NULL,							 /*	spiDeviceState			*/
					  baseAddress,					 /*	baseAddress			*/
					  0x00,							 /*	minAddress			*/
					  0x2B,							 /*	maxAddress			*/
					  repetitionsPerAddress,		 /*	repetitionsPerAddress		*/
					  chunkReadsPerAddress,			 /*	chunkReadsPerAddress		*/
					  spinDelay,					 /*	spinDelay			*/
					  autoIncrement,				 /*	autoIncrement			*/
					  sssupplyMillivolts,			 /*	sssupplyMillivolts		*/
					  referenceByte,				 /*	referenceByte			*/
					  adaptiveSssupplyMaxMillivolts, /*	adaptiveSssupplyMaxMillivolts	*/
					  chatty						 /*	chatty				*/
		);
#else
		SEGGER_RTT_WriteString(0, "\r\n\tAS7262 Read Aborted. Device Disabled :( ");
#endif
		break;
	}

	case kWarpSensorAS7263:
	{
		/*
			 *	AS7263: VDD 2.7--3.6
			 */
#ifdef WARP_BUILD_ENABLE_DEVAS7263
		loopForSensor("\r\nAS7263:\n\r",			 /*	tagString			*/
					  &readSensorRegisterAS7263,	 /*	readSensorRegisterFunction	*/
					  &deviceAS7263State,			 /*	i2cDeviceState			*/
					  NULL,							 /*	spiDeviceState			*/
					  baseAddress,					 /*	baseAddress			*/
					  0x00,							 /*	minAddress			*/
					  0x2B,							 /*	maxAddress			*/
					  repetitionsPerAddress,		 /*	repetitionsPerAddress		*/
					  chunkReadsPerAddress,			 /*	chunkReadsPerAddress		*/
					  spinDelay,					 /*	spinDelay			*/
					  autoIncrement,				 /*	autoIncrement			*/
					  sssupplyMillivolts,			 /*	sssupplyMillivolts		*/
					  referenceByte,				 /*	referenceByte			*/
					  adaptiveSssupplyMaxMillivolts, /*	adaptiveSssupplyMaxMillivolts	*/
					  chatty						 /*	chatty				*/
		);
#else
		SEGGER_RTT_WriteString(0, "\r\n\tAS7263 Read Aborted. Device Disabled :( ");
#endif
		break;
	}

	case kWarpSensorINA219:
	{
		/*
			 *	INA219
			 */
#ifdef WARP_BUILD_ENABLE_DEVINA219
		loopForSensorWithNumberOfBytes("\r\nINA219:\n\r",			  /*	tagString			*/
									   &readSensorRegisterINA219,	 /*	readSensorRegisterFunction	*/
									   &deviceINA219State,			  /*	i2cDeviceState			*/
									   NULL,						  /*	spiDeviceState			*/
									   baseAddress,					  /*	baseAddress			*/
									   0x00,						  /*	minAddress			*/
									   0x05,						  /*	maxAddress			*/
									   repetitionsPerAddress,		  /*	repetitionsPerAddress		*/
									   chunkReadsPerAddress,		  /*	chunkReadsPerAddress		*/
									   spinDelay,					  /*	spinDelay			*/
									   autoIncrement,				  /*	autoIncrement			*/
									   sssupplyMillivolts,			  /*	sssupplyMillivolts		*/
									   referenceByte,				  /*	referenceByte			*/
									   adaptiveSssupplyMaxMillivolts, /*	adaptiveSssupplyMaxMillivolts	*/
									   chatty,						  /*	chatty				*/
									   2							  /*	numberOfBytes 		*/
		);
#else
		SEGGER_RTT_WriteString(0, "\r\n\tINA219 Read Aborted. Device Disabled :(");
#endif
		break;
	}

	default:
	{
#ifdef WARP_BUILD_ENABLE_SEGGER_RTT_PRINTF
		SEGGER_RTT_printf(0, "\r\tInvalid warpSensorDevice [%d] passed to repeatRegisterReadForDeviceAndAddress.\n", warpSensorDevice);
#endif
	}
	}

	if (warpSensorDevice != kWarpSensorADXL362)
	{
		disableI2Cpins();
	}
}

int char2int(int character)
{
	if (character >= '0' && character <= '9')
	{
		return character - '0';
	}

	if (character >= 'a' && character <= 'f')
	{
		return character - 'a' + 10;
	}

	if (character >= 'A' && character <= 'F')
	{
		return character - 'A' + 10;
	}

	return 0;
}

uint8_t
readHexByte(void)
{
	uint8_t topNybble, bottomNybble;

	topNybble = SEGGER_RTT_WaitKey();
	bottomNybble = SEGGER_RTT_WaitKey();

	return (char2int(topNybble) << 4) + char2int(bottomNybble);
}

int read4digits(void)
{
	uint8_t digit1, digit2, digit3, digit4;

	digit1 = SEGGER_RTT_WaitKey();
	digit2 = SEGGER_RTT_WaitKey();
	digit3 = SEGGER_RTT_WaitKey();
	digit4 = SEGGER_RTT_WaitKey();

	return (digit1 - '0') * 1000 + (digit2 - '0') * 100 + (digit3 - '0') * 10 + (digit4 - '0');
}

WarpStatus
writeByteToI2cDeviceRegister(uint8_t i2cAddress, bool sendCommandByte, uint8_t commandByte, bool sendPayloadByte, uint8_t payloadByte)
{
	i2c_status_t status;
	uint8_t commandBuffer[1];
	uint8_t payloadBuffer[1];
	i2c_device_t i2cSlaveConfig =
		{
			.address = i2cAddress,
			.baudRate_kbps = gWarpI2cBaudRateKbps};

	commandBuffer[0] = commandByte;
	payloadBuffer[0] = payloadByte;

	status = I2C_DRV_MasterSendDataBlocking(
		0 /* instance */,
		&i2cSlaveConfig,
		commandBuffer,
		(sendCommandByte ? 1 : 0),
		payloadBuffer,
		(sendPayloadByte ? 1 : 0),
		gWarpI2cTimeoutMilliseconds);

	return (status == kStatus_I2C_Success ? kWarpStatusOK : kWarpStatusDeviceCommunicationFailed);
}
