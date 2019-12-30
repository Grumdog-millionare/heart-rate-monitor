#include <stdint.h>

#include "fsl_spi_master_driver.h"
#include "fsl_port_hal.h"

#include "SEGGER_RTT.h"
#include "gpio_pins.h"
#include "warp.h"
#include "devSSD1331.h"

volatile uint8_t inBuffer[1];
volatile uint8_t payloadBytes[1];

static int
writeCommand(uint8_t commandByte)
{
	spi_status_t status;

	/*
	 *	Drive /CS low.
	 *
	 *	Make sure there is a high-to-low transition by first driving high, delay, then drive low.
	 */
	GPIO_DRV_SetPinOutput(kSSD1331PinCSn);
	OSA_TimeDelay(0);
	GPIO_DRV_ClearPinOutput(kSSD1331PinCSn);

	/*
	 *	Drive DC low (command).
	 */
	GPIO_DRV_ClearPinOutput(kSSD1331PinDC);

	payloadBytes[0] = commandByte;
	status = SPI_DRV_MasterTransferBlocking(0 /* master instance */,
											NULL /* spi_master_user_config_t */,
											(const uint8_t *restrict) & payloadBytes[0],
											(uint8_t * restrict) & inBuffer[0],
											1 /* transfer size */,
											1000 /* timeout in microseconds (unlike I2C which is ms) */);

	/*
	 *	Drive /CS high
	 */
	GPIO_DRV_SetPinOutput(kSSD1331PinCSn);

	return status;
}

void clearScreen(void)
{
	writeCommand(kSSD1331CommandCLEAR);
	writeCommand(0x00);
	writeCommand(0x00);
	writeCommand(0x5F);
	writeCommand(0x3F);
	return;
}

void traceLine(uint8_t column, uint8_t prev, uint8_t next)
{
	writeCommand(kSSD1331CommandDRAWLINE);
	writeCommand(column);	// Column start address
	writeCommand(63 - prev); // Row start address (63 - because display is upside down)
	writeCommand(column);	// Column end address
	writeCommand(63 - next); // Row end address (63 - because display is upside down)
	writeCommand(0xFF);		 // Red
	writeCommand(0x00);		 // Green
	writeCommand(0x00);		 // Blue
}

// Draws digits in a 5x9 shaped box starting at the top left coordinates given
void writeDigit(uint8_t column, uint8_t row, uint8_t digit)
{
	row = 63 - row; // Screen is upside down
	switch (digit)
	{
	case 0:
	{
		writeCommand(kSSD1331CommandDRAWRECT);
		writeCommand(column);	 // Col start
		writeCommand(row);		  // Row start
		writeCommand(column + 4); // Col end
		writeCommand(row + 8);	// Row end
		writeCommand(0xFF);		  // Line red
		writeCommand(0xFF);		  // Line green
		writeCommand(0xFF);		  // Line blue
		writeCommand(0x00);		  // Fill red
		writeCommand(0x00);		  // Fill green
		writeCommand(0x00);		  // Fill blue

		break;
	}
	case 1:
	{
		writeCommand(kSSD1331CommandDRAWLINE);
		writeCommand(column + 2); // Column start address
		writeCommand(row);		  // Row start address
		writeCommand(column + 2); // Column end address
		writeCommand(row + 8);	// Row end address
		writeCommand(0xFF);		  // Red
		writeCommand(0xFF);		  // Green
		writeCommand(0xFF);		  // Blue

		break;
	}
	case 2:
	{
		for (int i = 0; i < 9; i += 4)
		{
			writeCommand(kSSD1331CommandDRAWLINE);
			writeCommand(column);	 // Column start address
			writeCommand(row + i);	// Row start address
			writeCommand(column + 4); // Column end address
			writeCommand(row + i);	// Row end address
			writeCommand(0xFF);		  // Red
			writeCommand(0xFF);		  // Green
			writeCommand(0xFF);		  // Blue
		}

		writeCommand(kSSD1331CommandDRAWLINE);
		writeCommand(column + 4); // Column start address
		writeCommand(row);		  // Row start address
		writeCommand(column + 4); // Column end address
		writeCommand(row + 4);	// Row end address
		writeCommand(0xFF);		  // Red
		writeCommand(0xFF);		  // Green
		writeCommand(0xFF);		  // Blue

		writeCommand(kSSD1331CommandDRAWLINE);
		writeCommand(column);  // Column start address
		writeCommand(row + 4); // Row start address
		writeCommand(column);  // Column end address
		writeCommand(row + 8); // Row end address
		writeCommand(0xFF);	// Red
		writeCommand(0xFF);	// Green
		writeCommand(0xFF);	// Blue

		break;
	}
	case 3:
	{
		for (int i = 0; i < 9; i += 4)
		{
			writeCommand(kSSD1331CommandDRAWLINE);
			writeCommand(column);	 // Column start address
			writeCommand(row + i);	// Row start address
			writeCommand(column + 4); // Column end address
			writeCommand(row + i);	// Row end address
			writeCommand(0xFF);		  // Red
			writeCommand(0xFF);		  // Green
			writeCommand(0xFF);		  // Blue
		}

		writeCommand(kSSD1331CommandDRAWLINE);
		writeCommand(column + 4); // Column start address
		writeCommand(row);		  // Row start address
		writeCommand(column + 4); // Column end address
		writeCommand(row + 8);	// Row end address
		writeCommand(0xFF);		  // Red
		writeCommand(0xFF);		  // Green
		writeCommand(0xFF);		  // Blue

		break;
	}
	case 4:
	{
		writeCommand(kSSD1331CommandDRAWLINE);
		writeCommand(column);  // Column start address
		writeCommand(row);	 // Row start address
		writeCommand(column);  // Column end address
		writeCommand(row + 4); // Row end address
		writeCommand(0xFF);	// Red
		writeCommand(0xFF);	// Green
		writeCommand(0xFF);	// Blue

		writeCommand(kSSD1331CommandDRAWLINE);
		writeCommand(column);	 // Column start address
		writeCommand(row + 4);	// Row start address
		writeCommand(column + 4); // Column end address
		writeCommand(row + 4);	// Row end address
		writeCommand(0xFF);		  // Red
		writeCommand(0xFF);		  // Green
		writeCommand(0xFF);		  // Blue

		writeCommand(kSSD1331CommandDRAWLINE);
		writeCommand(column + 4); // Column start address
		writeCommand(row);		  // Row start address
		writeCommand(column + 4); // Column end address
		writeCommand(row + 8);	// Row end address
		writeCommand(0xFF);		  // Red
		writeCommand(0xFF);		  // Green
		writeCommand(0xFF);		  // Blue

		break;
	}

	case 5:
	{
		for (int i = 0; i < 9; i += 4)
		{
			writeCommand(kSSD1331CommandDRAWLINE);
			writeCommand(column);	 // Column start address
			writeCommand(row + i);	// Row start address
			writeCommand(column + 4); // Column end address
			writeCommand(row + i);	// Row end address
			writeCommand(0xFF);		  // Red
			writeCommand(0xFF);		  // Green
			writeCommand(0xFF);		  // Blue
		}

		writeCommand(kSSD1331CommandDRAWLINE);
		writeCommand(column);  // Column start address
		writeCommand(row);	 // Row start address
		writeCommand(column);  // Column end address
		writeCommand(row + 4); // Row end address
		writeCommand(0xFF);	// Red
		writeCommand(0xFF);	// Green
		writeCommand(0xFF);	// Blue

		writeCommand(kSSD1331CommandDRAWLINE);
		writeCommand(column + 4); // Column start address
		writeCommand(row + 4);	// Row start address
		writeCommand(column + 4); // Column end address
		writeCommand(row + 8);	// Row end address
		writeCommand(0xFF);		  // Red
		writeCommand(0xFF);		  // Green
		writeCommand(0xFF);		  // Blue

		break;
	}
	case 6:
	{
		for (int i = 0; i < 9; i += 4)
		{
			writeCommand(kSSD1331CommandDRAWLINE);
			writeCommand(column);	 // Column start address
			writeCommand(row + i);	// Row start address
			writeCommand(column + 4); // Column end address
			writeCommand(row + i);	// Row end address
			writeCommand(0xFF);		  // Red
			writeCommand(0xFF);		  // Green
			writeCommand(0xFF);		  // Blue
		}

		writeCommand(kSSD1331CommandDRAWLINE);
		writeCommand(column);  // Column start address
		writeCommand(row);	 // Row start address
		writeCommand(column);  // Column end address
		writeCommand(row + 8); // Row end address
		writeCommand(0xFF);	// Red
		writeCommand(0xFF);	// Green
		writeCommand(0xFF);	// Blue

		writeCommand(kSSD1331CommandDRAWLINE);
		writeCommand(column + 4); // Column start address
		writeCommand(row + 4);	// Row start address
		writeCommand(column + 4); // Column end address
		writeCommand(row + 8);	// Row end address
		writeCommand(0xFF);		  // Red
		writeCommand(0xFF);		  // Green
		writeCommand(0xFF);		  // Blue

		break;
	}
	case 7:
	{

		writeCommand(kSSD1331CommandDRAWLINE);
		writeCommand(column);	 // Column start address
		writeCommand(row);		  // Row start address
		writeCommand(column + 4); // Column end address
		writeCommand(row);		  // Row end address
		writeCommand(0xFF);		  // Red
		writeCommand(0xFF);		  // Green
		writeCommand(0xFF);		  // Blue

		writeCommand(kSSD1331CommandDRAWLINE);
		writeCommand(column + 4); // Column start address
		writeCommand(row);		  // Row start address
		writeCommand(column);	 // Column end address
		writeCommand(row + 8);	// Row end address
		writeCommand(0xFF);		  // Red
		writeCommand(0xFF);		  // Green
		writeCommand(0xFF);		  // Blue

		break;
	}
	case 8:
	{
		writeCommand(kSSD1331CommandDRAWRECT);
		writeCommand(column);	 // Col start
		writeCommand(row);		  // Row start
		writeCommand(column + 4); // Col end
		writeCommand(row + 8);	// Row end
		writeCommand(0xFF);		  // Line red
		writeCommand(0xFF);		  // Line green
		writeCommand(0xFF);		  // Line blue
		writeCommand(0x00);		  // Fill red
		writeCommand(0x00);		  // Fill green
		writeCommand(0x00);		  // Fill blue

		writeCommand(kSSD1331CommandDRAWLINE);
		writeCommand(column);	 // Column start address
		writeCommand(row + 4);	// Row start address
		writeCommand(column + 4); // Column end address
		writeCommand(row + 4);	// Row end address
		writeCommand(0xFF);		  // Red
		writeCommand(0xFF);		  // Green
		writeCommand(0xFF);		  // Blue

		break;
	}
	case 9:
	{
		writeCommand(kSSD1331CommandDRAWRECT);
		writeCommand(column);	 // Col start
		writeCommand(row);		  // Row start
		writeCommand(column + 4); // Col end
		writeCommand(row + 4);	// Row end
		writeCommand(0xFF);		  // Line red
		writeCommand(0xFF);		  // Line green
		writeCommand(0xFF);		  // Line blue
		writeCommand(0x00);		  // Fill red
		writeCommand(0x00);		  // Fill green
		writeCommand(0x00);		  // Fill blue

		writeCommand(kSSD1331CommandDRAWLINE);
		writeCommand(column + 4); // Column start address
		writeCommand(row + 4);	// Row start address
		writeCommand(column + 4); // Column end address
		writeCommand(row + 8);	// Row end address
		writeCommand(0xFF);		  // Red
		writeCommand(0xFF);		  // Green
		writeCommand(0xFF);		  // Blue

		break;
	}
	}
	return;
}

// Writes a character symbol in a 5x9 shaped box starting at the top left
void writeCharacter(uint8_t column, uint8_t row, char character)
{
	row = 63 - row; // Screen is upside down
	switch (character)
	{
	case 'o':
	{
		writeCommand(kSSD1331CommandDRAWRECT);
		writeCommand(column);	 // Col start
		writeCommand(row);		  // Row start
		writeCommand(column + 3); // Col end
		writeCommand(row + 3);	// Row end
		writeCommand(0xFF);		  // Line red
		writeCommand(0xFF);		  // Line green
		writeCommand(0xFF);		  // Line blue
		writeCommand(0x00);		  // Fill red
		writeCommand(0x00);		  // Fill green
		writeCommand(0x00);		  // Fill blue
	}
	}
	return;
}

int devSSD1331init(void)
{
	/*
	 *	Configure as GPIO.
	 */
	PORT_HAL_SetMuxMode(PORTB_BASE, 13u, kPortMuxAsGpio);
	PORT_HAL_SetMuxMode(PORTA_BASE, 12u, kPortMuxAsGpio);
	PORT_HAL_SetMuxMode(PORTB_BASE, 0u, kPortMuxAsGpio);

	/*
	 *	RST high->low->high.
	 */
	GPIO_DRV_SetPinOutput(kSSD1331PinRST);
	OSA_TimeDelay(100);
	GPIO_DRV_ClearPinOutput(kSSD1331PinRST);
	OSA_TimeDelay(100);
	GPIO_DRV_SetPinOutput(kSSD1331PinRST);
	OSA_TimeDelay(100);

	/*
	 *	Initialization sequence, borrowed from https://github.com/adafruit/Adafruit-SSD1331-OLED-Driver-Library-for-Arduino
	 */
	writeCommand(kSSD1331CommandDISPLAYOFF); // 0xAE
	writeCommand(kSSD1331CommandSETREMAP);   // 0xA0
	writeCommand(0x72);						 // RGB Color
	writeCommand(kSSD1331CommandSTARTLINE);  // 0xA1
	writeCommand(0x0);
	writeCommand(kSSD1331CommandDISPLAYOFFSET); // 0xA2
	writeCommand(0x0);
	writeCommand(kSSD1331CommandNORMALDISPLAY); // 0xA4
	writeCommand(kSSD1331CommandSETMULTIPLEX);  // 0xA8
	writeCommand(0x3F);							// 0x3F 1/64 duty
	writeCommand(kSSD1331CommandSETMASTER);		// 0xAD
	writeCommand(0x8E);
	writeCommand(kSSD1331CommandPOWERMODE); // 0xB0
	writeCommand(0x0B);
	writeCommand(kSSD1331CommandPRECHARGE); // 0xB1
	writeCommand(0x31);
	writeCommand(kSSD1331CommandCLOCKDIV);   // 0xB3
	writeCommand(0xF0);						 // 7:4 = Oscillator Frequency, 3:0 = CLK Div Ratio (A[3:0]+1 = 1..16)
	writeCommand(kSSD1331CommandPRECHARGEA); // 0x8A
	writeCommand(0x64);
	writeCommand(kSSD1331CommandPRECHARGEB); // 0x8B
	writeCommand(0x78);
	writeCommand(kSSD1331CommandPRECHARGEA); // 0x8C
	writeCommand(0x64);
	writeCommand(kSSD1331CommandPRECHARGELEVEL); // 0xBB
	writeCommand(0x3A);
	writeCommand(kSSD1331CommandVCOMH); // 0xBE
	writeCommand(0x3E);
	writeCommand(kSSD1331CommandMASTERCURRENT); // 0x87
	writeCommand(0x0F);
	writeCommand(kSSD1331CommandCONTRASTA); // 0x81
	writeCommand(0x91);
	writeCommand(kSSD1331CommandCONTRASTB); // 0x82
	writeCommand(0x50);
	writeCommand(kSSD1331CommandCONTRASTC); // 0x83
	writeCommand(0x7D);
	writeCommand(kSSD1331CommandDISPLAYON); // Turn on oled panel

	/*
	 *	To use fill commands, you will have to issue a command to the display to enable them. See the manual.
	 */
	writeCommand(kSSD1331CommandFILL);
	writeCommand(0x01);

	clearScreen();

	/*
	 *	Any post-initialization drawing commands go here.
	 */
	//...
	// writeCommand(kSSD1331CommandDRAWRECT);
	// writeCommand(0);	// Col start
	// writeCommand(0);	// Row start
	// writeCommand(95);   // Col end
	// writeCommand(63);   // Row end
	// writeCommand(0x00); // Line red
	// writeCommand(0xFF); // Line green
	// writeCommand(0x00); // Line blue
	// writeCommand(0x00); // Fill red
	// writeCommand(0xFF); // Fill green
	// writeCommand(0x00); // Fill blue

	return 0;
}
