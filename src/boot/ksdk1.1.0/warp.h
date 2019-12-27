#include "fsl_spi_master_driver.h"

typedef enum
{
	CommStatusOK = 0,
	CommStatusDeviceCommunicationFailed,
	CommStatusBadDeviceCommand,
} CommStatus;

typedef enum
{
	SampleOK = 0,
	SampleNotUpdated,
} SamplingStatus;

typedef struct
{
	uint8_t i2cAddress;
	uint8_t i2cBuffer[192]; // Maximum number of bytes in MAX30105's FIFO for 2 channels
} WarpI2CDeviceState;

typedef enum
{
	FIFO_WRITE = 0x04,
	FIFO_READ = 0x06,
	FIFO_DATA = 0x07, // Read from this register
	FIFO_CONFIG = 0x08,
	MODE_CONFIG = 0x09,
	SPO2_CONFIG = 0x0A,
	LED1_PULSE_AMPLITUDE = 0x0C,
	LED2_PULSE_AMPLITUDE = 0x0D,
	LED3_PULSE_AMPLITUDE = 0x0E,
	PROX_MODE_LED_PULSE_AMPLITUDE = 0x10,
	MULTI_LED_MODE_CONTROL_CONFIG = 0x11,
} MAX30105Register;

void enableI2Cpins(uint16_t pullupValue);
void disableI2Cpins(void);
void enableSPIpins(void);
void disableSPIpins(void);
