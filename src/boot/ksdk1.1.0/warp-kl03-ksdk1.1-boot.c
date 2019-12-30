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
#include "fsl_interrupt_manager.h"

#include "gpio_pins.h"
#include "SEGGER_RTT.h"
#include "warp.h"

#include "devSSD1331.h"
#include "devMAX30105.h"

#define WARP_BUILD_ENABLE_SEGGER_RTT_PRINTF

#define kWarpConstantStringI2cFailure "\rI2C failed, reg 0x%02x, code %d\n"
#define kWarpConstantStringErrorInvalidVoltage "\rInvalid supply voltage [%d] mV!"
#define kWarpConstantStringErrorSanity "\rSanity check failed!"

volatile WarpI2CDeviceState deviceMAX30105State;

volatile i2c_master_state_t i2cMasterState;
volatile spi_master_state_t spiMasterState;
volatile spi_master_user_config_t spiUserConfig;

volatile uint32_t gWarpI2cBaudRateKbps = 200;
volatile uint32_t gWarpUartBaudRateKbps = 1;
volatile uint32_t gWarpSpiBaudRateKbps = 200;
volatile uint32_t gWarpSleeptimeSeconds = 0;
volatile uint32_t gWarpI2cTimeoutMilliseconds = 5;
volatile uint32_t gWarpSpiTimeoutMicroseconds = 5;
volatile uint32_t gWarpMenuPrintDelayMilliseconds = 10;
volatile uint32_t gWarpSupplySettlingDelayMilliseconds = 1;

// CONSTANTS
const uint32_t THRESHOLD_UP = 1024;
const uint32_t THRESHOLD_DOWN = 2000;

// GLOBAL VARIABLES
volatile bool active;

void enableSPIpins(void)
{
	CLOCK_SYS_EnableSpiClock(0);

	// PTA8 -> MOSI
	PORT_HAL_SetMuxMode(PORTA_BASE, 8u, kPortMuxAlt3);

	// PTA9 -> SCK
	PORT_HAL_SetMuxMode(PORTA_BASE, 9u, kPortMuxAlt3);

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

	// PTA8 -> GPIO
	PORT_HAL_SetMuxMode(PORTA_BASE, 8u, kPortMuxAsGpio);

	// PTA9 -> GPIO
	PORT_HAL_SetMuxMode(PORTA_BASE, 9u, kPortMuxAsGpio);

	GPIO_DRV_ClearPinOutput(kSSD1331PinMOSI);
	GPIO_DRV_ClearPinOutput(kSSD1331PinSCK);

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
	GPIO_DRV_ClearPinOutput(kMAX30105PinI2C0_SDA);
	GPIO_DRV_ClearPinOutput(kMAX30105PinI2C0_SCL);

	CLOCK_SYS_DisableI2cClock(0);
}

void PORTA_IRQHandler(void)
{
	// Perhaps need to abstract out large code to another function due to little space in the vector table
	PORT_HAL_ClearPortIntFlag(PORTA_BASE);
	active = true;
	return;
}

int main(void)
{
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

	SEGGER_RTT_WriteString(0, "\n\n\n\rBooting Heart Rate Monitor, in 3... ");
	OSA_TimeDelay(200);
	SEGGER_RTT_WriteString(0, "2... ");
	OSA_TimeDelay(200);
	SEGGER_RTT_WriteString(0, "1...\n\r");
	OSA_TimeDelay(200);

	/*
	 *	Initialize the GPIO pins with the appropriate pull-up, etc.,
	 *	defined in the inputPins and outputPins arrays (gpio_pins.c).
	 *
	 *	See also Section 30.3.3 GPIO Initialization of KSDK13APIRM.pdf
	 */
	GPIO_DRV_Init(inputPins /* input pins */, outputPins /* output pins */);

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

	// Enable interrupt port
	PORT_HAL_SetMuxMode(PORTA_BASE, 7u, kPortMuxAsGpio);

	// Enable SPI and I2C
	enableSPIpins();
	enableI2Cpins(32768);

	// Initialize all devices
	devSSD1331init();
	devMAX30105init(0x57 /* i2cAddress */);

	// Configure MAX30105
	configureSensorMAX30105();

	// Variables
	active = false;
	uint8_t interrupt_status;

	uint16_t sample;
	uint16_t buffer[32];
	uint8_t buffer_pointer = 0;
	uint8_t buffer_size = 0;

	uint64_t filtered_sample;
	uint16_t filtered_buffer[256];
	uint8_t filtered_buffer_pointer = 0;
	uint16_t filtered_buffer_size = 0;
	uint32_t filtered_buffer_mean;
	uint16_t filtered_buffer_max;
	uint16_t filtered_buffer_min;

	int8_t previous_display_value = 0;
	int8_t display_value = 0;
	int8_t display_count = 0;

	uint16_t fir_coeffs[12] = {172, 321, 579, 927, 1360, 1858, 2390, 2916, 3391, 3768, 4012, 4096};

	// Read INTERRUPT_STATUS_1 to clear Power ready status
	readSensorRegisterMAX30105(INTERRUPT_STATUS_1, 1);
	interrupt_status = deviceMAX30105State.i2cBuffer[0];

	while (1)
	{
		while (active)
		{
			SamplingStatus readStatus = readNextSample(&sample);
			if (readStatus == SampleOK)
			{
				//SEGGER_RTT_printf(0, "Sample: %u\n", sample);
				// Check if finger has been removed
				if ((sample < THRESHOLD_DOWN))
				{
					// Reset mode
					writeSensorRegisterMAX30105(MODE_CONFIG, 0x03);
					// Read INTERRUPT_STATUS_1 to clear Power ready status
					readSensorRegisterMAX30105(INTERRUPT_STATUS_1, 1);
					interrupt_status = deviceMAX30105State.i2cBuffer[0];
					// Clear screen
					clearScreen();
					display_count = 0;

					active = false;
					buffer_pointer = 0;
					buffer_size = 0;
					filtered_buffer_pointer = 0;
					filtered_buffer_size = 0;
					break;
				}

				// Write sample to buffer
				buffer[buffer_pointer] = sample;

				if (buffer_size < 32)
				{
					buffer_size++;
					//SEGGER_RTT_printf(0, "Buffer being filled\n", 0);
				}
				else
				{
					// Low pass filter
					filtered_sample = fir_coeffs[11] * buffer[(buffer_pointer - 11) & 0x1F];
					// SEGGER_RTT_printf(0, "Pointer: %u	Middle pointer: %u\n", buffer_pointer, (buffer_pointer - 11) & 0x1F);
					for (int i = 0; i < 11; i++)
					{
						filtered_sample += fir_coeffs[i] * (buffer[(buffer_pointer - 21 + i) & 0x1F] + buffer[(buffer_pointer - i) & 0x1F]);
					}
					filtered_sample = filtered_sample >> 16;

					// Write filtered sample to filtered buffer
					filtered_buffer[filtered_buffer_pointer] = filtered_sample;
					filtered_buffer_pointer++;
					if (filtered_buffer_size < 256)
					{
						filtered_buffer_size++;
					}

					// Calculate normalised display value
					filtered_buffer_mean = 0;
					filtered_buffer_max = filtered_buffer[0];
					filtered_buffer_min = filtered_buffer[0];

					for (int i = 0; i < filtered_buffer_size; i++)
					{
						filtered_buffer_mean += filtered_buffer[i];
						if (filtered_buffer[i] > filtered_buffer_max)
						{
							filtered_buffer_max = filtered_buffer[i];
						}
						else if (filtered_buffer[i] < filtered_buffer_min)
						{
							filtered_buffer_min = filtered_buffer[i];
						}
					}
					filtered_buffer_mean /= filtered_buffer_size;
					SEGGER_RTT_printf(0, "Filtered sample: %u\t", filtered_sample);
					//SEGGER_RTT_printf(0, "Filtered buffer mean: %u\t", filtered_buffer_mean);
					SEGGER_RTT_printf(0, "Max: %u\t", filtered_buffer_max);
					SEGGER_RTT_printf(0, "Min: %u\t", filtered_buffer_min);
					SEGGER_RTT_printf(0, "Filtered buffer size: %u\n", filtered_buffer_size);

					previous_display_value = display_value;
					display_value = (filtered_sample - filtered_buffer_min) * 63 / (filtered_buffer_max - filtered_buffer_min); //(filtered_sample - filtered_buffer_mean) * 63 / 200;
					// if (display_value < 0)
					// {
					// 	display_value = 0;
					// }
					// else if (display_value > 63)
					// {
					// 	display_value = 63;
					// }

					// Write to display
					if (display_count > 95)
					{
						clearScreen();
						display_count = 0;
					}
					traceLine(display_count, previous_display_value, display_value);
					display_count++;
				}

				if (buffer_pointer < 31)
				{
					buffer_pointer++;
				}
				else
				{
					buffer_pointer = 0;
				}
			}
		}
	}
	return 0;
}
