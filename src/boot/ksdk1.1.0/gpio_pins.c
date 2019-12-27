#include <stdint.h>
#include <stdlib.h>
#include "gpio_pins.h"
#include "device/fsl_device_registers.h"

/*
 *	See Section 12.1.1 "GPIO instantiation information" of KL03 family reference, KL03P24M48SF0RM.pdf
 *	for the default state of pins, pull capability, etc.:
 *
 *		PTA0 : pulled down at reset
 *		PTA2 : pulled up at reset
 *		PTA1 / RESET_b : pulled up at reset
 *		PTB5 : pulled up at reset
 *
 *	See Section 5 of "Power Management for Kinetis L Family" (AN5088.pdf) for additional hints on pin setup for low power
 *
 *	Here, we configure all pins that we ever use as general-purpose output.
 *
 */

gpio_output_pin_user_config_t outputPins[] = {
	{
		.pinName = kWarpPinSI4705_nRST,
		.config.outputLogic = 1,
		.config.slewRate = kPortSlowSlewRate,
		.config.driveStrength = kPortLowDriveStrength,
	},
	{
		.pinName = kMAX30105PinI2C0_SCL,
		.config.outputLogic = 1,
		.config.slewRate = kPortSlowSlewRate,
		.config.driveStrength = kPortLowDriveStrength,
	},
	{
		.pinName = kMAX30105PinI2C0_SDA,
		.config.outputLogic = 1,
		.config.slewRate = kPortSlowSlewRate,
		.config.driveStrength = kPortLowDriveStrength,
	},
	{
		.pinName = kSSD1331PinDC,
		.config.outputLogic = 1,
		.config.slewRate = kPortSlowSlewRate,
		.config.driveStrength = kPortLowDriveStrength,
	},
	{
		.pinName = kSSD1331PinRST,
		.config.outputLogic = 1,
		.config.slewRate = kPortSlowSlewRate,
		.config.driveStrength = kPortLowDriveStrength,
	},
	{
		.pinName = kSSD1331PinCSn,
		.config.outputLogic = 1,
		.config.slewRate = kPortSlowSlewRate,
		.config.driveStrength = kPortLowDriveStrength,
	},
	{
		.pinName = GPIO_PINS_OUT_OF_RANGE,
	}};

/*
 *	Configuration to be passed to GPIO_DRV_Init() to disable all pins.
 *
 *	NOTE: the type here is
 *
 *			gpio_input_pin_user_config_t
 *	not
 *
 *			gpio_output_pin_user_config_t
 *
 *	like the above.
 *
 *	PTB1 is tied to VBATT. Need to configure it as an input pin.
 *
 */
gpio_input_pin_user_config_t inputPins[] = {
	{
		.pinName = kMAX30105PinINTERRUPT,
		.config.isPullEnable = true,
		.config.pullSelect = kPortPullUp,
		.config.isPassiveFilterEnabled = false,
		.config.interrupt = kPortIntFallingEdge,
	},
	{
		.pinName = GPIO_PINS_OUT_OF_RANGE,
	}};
