#ifndef __FSL_GPIO_PINS_H__
#define __FSL_GPIO_PINS_H__

#include "fsl_gpio_driver.h"

enum _gpio_pins
{
	kWarpPinSI4705_nRST = GPIO_MAKE_PIN(HW_GPIOB, 7), /*	Warp SI4705_nRST	--> PTB7		(was unused in Warp v2)					*/

	kMAX30105PinI2C0_SCL = GPIO_MAKE_PIN(HW_GPIOB, 3), /*	MAX30105 I2C SCL	--> PTB3									*/
	kMAX30105PinI2C0_SDA = GPIO_MAKE_PIN(HW_GPIOB, 4), /*	MAX30105 I2C SDA	--> PTB4									*/
	kMAX30105PinINTERRUPT = GPIO_MAKE_PIN(HW_GPIOA, 7),

	kSSD1331PinMOSI = GPIO_MAKE_PIN(HW_GPIOA, 8),
	kSSD1331PinSCK = GPIO_MAKE_PIN(HW_GPIOA, 9),
	kSSD1331PinDC = GPIO_MAKE_PIN(HW_GPIOA, 12),
	kSSD1331PinRST = GPIO_MAKE_PIN(HW_GPIOB, 0),
	kSSD1331PinCSn = GPIO_MAKE_PIN(HW_GPIOB, 13),
};

extern gpio_input_pin_user_config_t inputPins[];
extern gpio_output_pin_user_config_t outputPins[];

#endif /* __FSL_GPIO_PINS_H__ */
