#include "fsl_spi_master_driver.h"

#define min(x, y) ((x) < (y) ? (x) : (y))
#define USED(x) (void)(x)

typedef enum
{
	/*
	 *	Always keep these two as the last items.
	 */
	kWarpTypeMaskTime,
	kWarpTypeMaskMax,
} WarpTypeMask;

typedef enum
{
	/*
	 *	Always keep this as the last item.
	 */
	kWarpSignalPrecisionMax
} WarpSignalPrecision;

typedef enum
{
	/*
	 *	Always keep this as the last item.
	 */
	kWarpSignalAccuracyMax
} WarpSignalAccuracy;

typedef enum
{
	/*
	 *	Always keep this as the last item.
	 */
	kWarpSignalReliabilityMax
} WarpSignalReliability;

typedef enum
{
	/*
	 *	Always keep this as the last item.
	 */
	kWarpSignalNoiseMax
} WarpSignalNoise;

typedef enum
{
	kWarpStatusOK = 0,

	kWarpStatusDeviceNotInitialized,
	kWarpStatusDeviceCommunicationFailed,
	kWarpStatusBadDeviceCommand,

	/*
	 *	Generic comms error 
	 */
	kWarpStatusCommsError,

	/*
	 *	Power mode routines
	 */
	kWarpStatusPowerTransitionErrorVlpr2Wait,
	kWarpStatusPowerTransitionErrorVlpr2Stop,
	kWarpStatusPowerTransitionErrorRun2Vlpw,
	kWarpStatusPowerTransitionErrorVlpr2Vlpr,
	kWarpStatusErrorPowerSysSetmode,
	kWarpStatusBadPowerModeSpecified,

	/*
	 *	Always keep this as the last item.
	 */
	kWarpStatusMax
} WarpStatus;

typedef enum
{
	/*
	 *	NOTE: This order is depended on by POWER_SYS_SetMode()
	 *
	 *	See KSDK13APIRM.pdf Section 55.5.3
	 */
	kWarpPowerModeWAIT,
	kWarpPowerModeSTOP,
	kWarpPowerModeVLPR,
	kWarpPowerModeVLPW,
	kWarpPowerModeVLPS,
	kWarpPowerModeVLLS0,
	kWarpPowerModeVLLS1,
	kWarpPowerModeVLLS3,
	kWarpPowerModeRUN,
} WarpPowerMode;

typedef enum
{
	kWarpSensorADXL362,
	kWarpSensorMMA8451Q,
	kWarpSensorBME680,
	kWarpSensorBMX055accel,
	kWarpSensorBMX055gyro,
	kWarpSensorBMX055mag,
	kWarpSensorTMP006B,
	kWarpSensorMAG3110,
	kWarpSensorL3GD20H,
	kWarpSensorLPS25H,
	kWarpSensorTCS34725,
	kWarpSensorSI4705,
	kWarpSensorHDC1000,
	kWarpSensorSI7021,
	kWarpSensorAMG8834,
	kWarpSensorCCS811,
	kWarpSensorPAN1326,
	kWarpSensorAS7262,
	kWarpSensorAS7263,
	kWarpSensorSCD30,
	kWarpSensorINA219,
} WarpSensorDevice;

typedef enum
{
	kWarpModeDisableAdcOnSleep = (1 << 0),
} WarpModeMask;

typedef enum
{
	kWarpSizesI2cBufferBytes = 4,
	kWarpSizesSpiBufferBytes = 3,
	kWarpSizesBME680CalibrationValuesCount = 41,
} WarpSizes;

typedef struct
{
	uint8_t i2cAddress;
	WarpTypeMask signalType;
	uint8_t i2cBuffer[kWarpSizesI2cBufferBytes];

	WarpStatus deviceStatus;
} WarpI2CDeviceState;

typedef enum
{
	kWarpSensorConfigurationRegisterINA219Configuration = 0x00,
	kWarpSensorConfigurationRegisterINA219Calibration = 0x05,
} WarpSensorConfigurationRegister;

typedef enum
{
	kWarpSensorOutputRegisterINA219Configuration = 0x00,
	kWarpSensorOutputRegisterINA219Current = 0x04,
	kWarpSensorOutputRegisterINA219Calibration = 0x05,
} WarpSensorOutputRegister;

typedef struct
{
	/*
	 *	For holding ksdk-based error codes
	 */
	spi_status_t ksdk_spi_status;

	WarpTypeMask signalType;

	uint8_t spiSourceBuffer[kWarpSizesSpiBufferBytes];
	uint8_t spiSinkBuffer[kWarpSizesSpiBufferBytes];
	WarpStatus deviceStatus;
} WarpSPIDeviceState;

typedef struct
{
	WarpTypeMask signalType;
	WarpStatus deviceStatus;
} WarpUARTDeviceState;

typedef struct
{
	uint8_t errorCount;
} WarpPowerManagerCallbackStructure;

typedef enum
{
	kWarpThermalChamberMemoryFillEvenComponent = 0b00110011,
	kWarpThermalChamberMemoryFillOddComponent = 0b11001100,
	kWarpThermalChamberMMA8451QOutputBufferSize = 3,
	kWarpThermalChamberKL03MemoryFillBufferSize = 200,
	kWarpThermalChamberBusyLoopCountOffset = 65535,
	kWarpThermalChamberBusyLoopAdder = 99,
	kWarpThermalChamberBusyLoopMutiplier = 254,
} WarpThermalChamber;

typedef struct
{
	/*
	 *	Fill up the remaining memory space using an array
	 *	The size of the array is highly dependent on
	 *	the firmware code size
	 */
	uint8_t memoryFillingBuffer[kWarpThermalChamberKL03MemoryFillBufferSize];
	uint8_t outputBuffer[kWarpThermalChamberMMA8451QOutputBufferSize];
} WarpThermalChamberKL03MemoryFill;

WarpStatus warpSetLowPowerMode(WarpPowerMode powerMode, uint32_t sleepSeconds);
void enableI2Cpins(uint16_t pullupValue);
void disableI2Cpins(void);
void enableSPIpins(void);
void disableSPIpins(void);
