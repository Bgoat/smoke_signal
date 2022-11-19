// ConsoleCommands.c
// This is where you add commands:
//		1. Add a protoype
//			static eCommandResult_T ConsoleCommandVer(const char buffer[]);
//		2. Add the command to mConsoleCommandTable
//		    {"ver", &ConsoleCommandVer, HELP("Get the version string")},
//		3. Implement the function, using ConsoleReceiveParam<Type> to get the parameters from the buffer.

#include <string.h>
#include "consoleCommands.h"
#include "console.h"
#include "consoleIo.h"
#include "version.h"
#include "../../Drivers/BSP/STM32F411E-Discovery/stm32f411e_discovery_accelerometer.h"
#include "../../Drivers/BSP/STM32F411E-Discovery/stm32f411e_discovery_gyroscope.h"
#include "tim.h"
#include "main.h"
#include "gpio.h"

#define IGNORE_UNUSED_VARIABLE(x)     if ( &x == &x ) {}


static eCommandResult_T ConsoleCommandComment(const char buffer[]);
static eCommandResult_T ConsoleCommandVer(const char buffer[]);
static eCommandResult_T ConsoleCommandHelp(const char buffer[]);
static eCommandResult_T ConsoleCommandAccel(const char buffer[]);
static eCommandResult_T ConsoleCommandGyro(const char buffer[]);
static eCommandResult_T ConsoleCommandFlashRW(const char buffer[]);
static eCommandResult_T ConsoleCommandSonar(const char buffer[]);
static eCommandResult_T ConsoleCommandParamExampleInt16(const char buffer[]);
static eCommandResult_T ConsoleCommandParamExampleHexUint16(const char buffer[]);


static const sConsoleCommandTable_T mConsoleCommandTable[] =
{
    {";", &ConsoleCommandComment, HELP("Comment! You do need a space after the semicolon. ")},
    {"help", &ConsoleCommandHelp, HELP("Lists the commands available")},
    {"accel", &ConsoleCommandAccel, HELP("Testing the accel dims")},
    {"gyro", &ConsoleCommandGyro, HELP("Testing the gyro movement")},
    {"flash", &ConsoleCommandFlashRW, HELP("Testing the Flash read and write")},
    {"sonar", &ConsoleCommandSonar, HELP("Getting Sonar Distance")},
    {"ver", &ConsoleCommandVer, HELP("Get the version string")},
    {"int", &ConsoleCommandParamExampleInt16, HELP("How to get a signed int16 from params list: int -321")},
    {"u16h", &ConsoleCommandParamExampleHexUint16, HELP("How to get a hex u16 from the params list: u16h aB12")},

	CONSOLE_COMMAND_TABLE_END // must be LAST
};

static eCommandResult_T ConsoleCommandComment(const char buffer[])
{
	// do nothing
	IGNORE_UNUSED_VARIABLE(buffer);
	return COMMAND_SUCCESS;
}

static eCommandResult_T ConsoleCommandHelp(const char buffer[])
{
	uint32_t i;
	uint32_t tableLength;
	eCommandResult_T result = COMMAND_SUCCESS;

    IGNORE_UNUSED_VARIABLE(buffer);

	tableLength = sizeof(mConsoleCommandTable) / sizeof(mConsoleCommandTable[0]);
	for ( i = 0u ; i < tableLength - 1u ; i++ )
	{
		ConsoleIoSendString(mConsoleCommandTable[i].name);
#if CONSOLE_COMMAND_MAX_HELP_LENGTH > 0
		ConsoleIoSendString(" : ");
		ConsoleIoSendString(mConsoleCommandTable[i].help);
#endif // CONSOLE_COMMAND_MAX_HELP_LENGTH > 0
		ConsoleIoSendString(STR_ENDLINE);
	}
	return result;
}
static eCommandResult_T ConsoleCommandAccel(const char buffer[])
{
	int16_t data_xyz[3];
	BSP_ACCELERO_GetXYZ(&data_xyz);
	eCommandResult_T result = COMMAND_SUCCESS;
	if ( COMMAND_SUCCESS == result )
	{
		ConsoleIoSendString("X is ");
		ConsoleSendParamInt16(data_xyz[0]);
		ConsoleIoSendString(" Y is ");
		ConsoleSendParamInt16(data_xyz[1]);
		ConsoleIoSendString(" Z is ");
		ConsoleSendParamInt16(data_xyz[2]);
		ConsoleIoSendString(" ");
		ConsoleIoSendString(STR_ENDLINE);
	}
	return result;
}
static eCommandResult_T ConsoleCommandGyro(const char buffer[])
{
	int16_t data_xyz[3];
	BSP_GYRO_GetXYZ(&data_xyz);
	eCommandResult_T result = COMMAND_SUCCESS;
	if ( COMMAND_SUCCESS == result )
	{
		ConsoleIoSendString("X is ");
		ConsoleSendParamInt16(data_xyz[0]);
		ConsoleIoSendString(" Y is ");
		ConsoleSendParamInt16(data_xyz[1]);
		ConsoleIoSendString(" Z is ");
		ConsoleSendParamInt16(data_xyz[2]);
		ConsoleIoSendString(" ");
		ConsoleIoSendString(STR_ENDLINE);
	}
	return result;
}
static eCommandResult_T ConsoleCommandFlashRW(const char buffer[])
{
	#define FLASH_FREE_LOCATION 0x08020000
	//read first to make sure we are good
	uint64_t *RDAddr = (uint64_t *) FLASH_FREE_LOCATION;
	uint64_t RData = *RDAddr;
	uint32_t Rdata_bottom = (uint32_t)(RData & 0xFFFFFFFFLL);
	uint32_t Rdata_top = (uint32_t)((RData & 0xFFFFFFFF00000000LL) >> 32);
	ConsoleIoSendString("Data is ");
	ConsoleSendParamInt32(Rdata_top);
	ConsoleSendParamInt32(Rdata_bottom);
	ConsoleIoSendString(STR_ENDLINE);


 	HAL_FLASH_Unlock();
 	//HAL_FLASH_OB_Unlock();
 	// Erase
 	//Instantiate the FLASH_EraseInitTypeDef struct needed for the HAL_FLASHEx_Erase() function
	FLASH_EraseInitTypeDef FLASH_EraseInitStruct = {0};

	FLASH_EraseInitStruct.TypeErase = FLASH_TYPEERASE_SECTORS;  //Erase type set to sectors
	FLASH_EraseInitStruct.Sector = 5;            				//sector 0x0800 C000
	FLASH_EraseInitStruct.NbSectors = 1;                        //The number of sectors
	FLASH_EraseInitStruct.VoltageRange = FLASH_VOLTAGE_RANGE_3;

	uint32_t  errorStatus = 0;

	HAL_FLASHEx_Erase(&FLASH_EraseInitStruct,&errorStatus);
	HAL_Delay(10);

 	uint8_t FData = 0xAA;
 	// FLASH->CR &= (FLASH_CR_PG);

 	HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE,0x08020000, FData);
 	HAL_Delay(10);
 	HAL_FLASH_Lock();

 	RData = *RDAddr;
	Rdata_bottom = (uint32_t)(RData & 0xFFFFFFFFLL);
	Rdata_top = (uint32_t)((RData & 0xFFFFFFFF00000000LL) >> 32);
	ConsoleIoSendString("Data is now ");
	ConsoleSendParamInt32(Rdata_top);
	ConsoleSendParamInt32(Rdata_bottom);
	ConsoleIoSendString(STR_ENDLINE);
 	eCommandResult_T result = COMMAND_SUCCESS;

	return result;
}

static eCommandResult_T ConsoleCommandSonar(const char buffer[])
{
	uint8_t Distance = 200;
	ConsoleIoSendString("Distance is ");

	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_8, GPIO_PIN_SET);  // pull the TRIG pin HIGH
	delay_us(10);  // wait for 10 us
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_8, GPIO_PIN_RESET);  // pull the TRIG pin low

	__HAL_TIM_ENABLE_IT(&htim1, TIM_IT_CC1);
	HAL_Delay(100);
	ConsoleSendParamInt16(Distance);
	ConsoleIoSendString(STR_ENDLINE);

 	eCommandResult_T result = COMMAND_SUCCESS;

	return result;
}



static eCommandResult_T ConsoleCommandParamExampleInt16(const char buffer[])
{
	int16_t parameterInt;
	eCommandResult_T result;
	result = ConsoleReceiveParamInt16(buffer, 1, &parameterInt);
	if ( COMMAND_SUCCESS == result )
	{
		ConsoleIoSendString("Parameter is ");
		ConsoleSendParamInt16(parameterInt);
		ConsoleIoSendString(" (0x");
		ConsoleSendParamHexUint16((uint16_t)parameterInt);
		ConsoleIoSendString(")");
		ConsoleIoSendString(STR_ENDLINE);
	}
	return result;
}
static eCommandResult_T ConsoleCommandParamExampleHexUint16(const char buffer[])
{
	uint16_t parameterUint16;
	eCommandResult_T result;
	result = ConsoleReceiveParamHexUint16(buffer, 1, &parameterUint16);
	if ( COMMAND_SUCCESS == result )
	{
		ConsoleIoSendString("Parameter is 0x");
		ConsoleSendParamHexUint16(parameterUint16);
		ConsoleIoSendString(STR_ENDLINE);
	}
	return result;
}

static eCommandResult_T ConsoleCommandVer(const char buffer[])
{
	eCommandResult_T result = COMMAND_SUCCESS;

    IGNORE_UNUSED_VARIABLE(buffer);

	ConsoleIoSendString(VERSION_STRING);
	ConsoleIoSendString(STR_ENDLINE);
	return result;
}


const sConsoleCommandTable_T* ConsoleCommandsGetTable(void)
{
	return (mConsoleCommandTable);
}


