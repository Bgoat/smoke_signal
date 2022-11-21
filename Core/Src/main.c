/* USER CODE BEGIN Header */
/**
 * William Goethals
 * Nov 2022 for MES
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "i2c.h"
#include "i2s.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "input_buf.h"
#include "console.h"
#include <stdbool.h>
#include "../../Drivers/BSP/STM32F411E-Discovery/stm32f411e_discovery_accelerometer.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
// Flash in STM32F411 is from 0x0800 0000 to 0x0807 FFFF
#define FLASH_START 0x08009000

//State for operation and headings
enum operation_mode {mode_cli = 0, mode_local = 1, mode_moving = 2, mode_sensing = 3, mode_measure = 4};
enum operation_mode op_mode = mode_cli;
bool button_pressed = false;
enum heading{north =0, east=1, south = 2, west =3};
enum heading current_heading = north;

// Movement vars
bool block = false;


// CLI vars
uint8_t uart_byte_buf[1];
input_buf uart_buf;
char c;
bool echo;
char buf[20];
extern char mReceiveBuffer[256]; // should be CONSOLE_COMMAND_MAX_LENGTH, but it's in consoleCommands.h
extern uint32_t mReceivedSoFar;

// Button Vars
uint32_t previousMs = 0;
uint32_t currentMs = 0;

// timer vars for Sonar
uint32_t IC_Val1 = 0;
uint32_t IC_Val2 = 0;
uint32_t Difference = 0;
uint8_t Is_First_Captured = 0;  // is the first value captured ?
uint8_t Distance   = 0;
uint8_t DistanceR  = 0;
uint8_t DistanceL  = 0;
uint32_t wall_space = 30; // Start with the wall ~ 1 foot away and move it

// Motor Vars
const uint16_t turn_time = 800;
const uint16_t forward_time = 500;
const uint32_t speed = 30000;
const uint32_t speed_R = speed*.82;

// Gas and location vars
uint16_t gas_values[30][30]; // setting larger than testing space - to change to dynamically allocated Array in FUTURE
uint32_t x_loc = 0;
uint32_t y_loc = 0;
uint32_t x_ori = 0;
uint32_t y_ori = 0;

uint16_t rawADC;

// Settings for just doing a perimeter or small demo movement
bool perimeter_only = true;
bool demo_mode = false;

#define TRIG_PINL GPIO_PIN_7
#define TRIG_PINR GPIO_PIN_10
#define TRIG_PIN GPIO_PIN_8
#define TRIG_PORT GPIOE
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
/* USER CODE BEGIN PFP */
void cli_check(void);
void delay_us(uint16_t us);
void HCSR04_Read(void);
void HCSR04_ReadRight(void);
void HCSR04_ReadLeft(void);

void straight(uint16_t ms_time);
void right(uint16_t ms_time);
void left(uint16_t ms_time);
void back(uint16_t ms_time);
void wall_follow(void);
uint16_t read_gas(void);
void demo_drive(void);
void check_sonar(void);
void linear_reg(int n);
void flash_write(uint64_t data);
void origin_check(void);
void update_heading(void);
void update_heading_back(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// uart interrupt callback
// huart2 defined in usart.c, generated from cubemx
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	c = uart_byte_buf[0];
	echo = true;
	input_buf_add(&uart_buf, uart_byte_buf[0]);
	HAL_UART_Receive_IT(&huart2, uart_byte_buf, 1);
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

/* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */
  BSP_ACCELERO_Init();
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_I2S2_Init();
  MX_I2S3_Init();
  MX_SPI1_Init();
  MX_USART2_UART_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
  ConsoleInit();
  input_buf_reset(&uart_buf);
  HAL_UART_Receive_IT(&huart2, uart_byte_buf, 1);
  HAL_TIM_Base_Start(&htim1);
  HAL_TIM_Base_Start(&htim2);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
  HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_1);
  HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_2);
  HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_3);
  // For running motors comment for now
  //HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  switch(op_mode)
	  	  {
	  	  case mode_cli:
	  		  cli_check();
	  			if(button_pressed != 0)
	  			{
	  				op_mode = mode_local;
	  			}
	  		  break;
	  	  case mode_local:
	  		  //find where I am if first time
	  		  // eCompass to set direction, for now, origin is North relative
	  		  current_heading = north;
	  		  op_mode = mode_moving;
	  		  break;
	  	  case mode_moving:
	  		  if(demo_mode)
	  		  {
	  			  demo_drive();
	  		  }
	  		  else
	  		  {
	  			  check_sonar();
	  			  wall_follow();
	  		  }
	  		  break;
	  	  case mode_sensing:
	  		  origin_check();
	  		  gas_values[x_loc][y_loc] = read_gas();
	  		  op_mode = mode_moving;
	  		  break;
	  	  case mode_measure:
	  		  linear_reg(2);
	  		  cli_check();
	  		  break;


	  	  }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 8;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_I2S;
  PeriphClkInitStruct.PLLI2S.PLLI2SN = 200;
  PeriphClkInitStruct.PLLI2S.PLLI2SM = 5;
  PeriphClkInitStruct.PLLI2S.PLLI2SR = 2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void cli_check(void)
{
 	if(echo) {
		sprintf((char*)buf, "%c", c);
  		HAL_UART_Transmit(&huart2, buf, 1, HAL_MAX_DELAY);
  		echo = false;
  	}

  	if(input_buf_ready(&uart_buf)) {
  		// remove the c in sprintf((char*)buf, "\r\n", c);
  		sprintf((char*)buf, "\r\n", c);
  		HAL_UART_Transmit(&huart2, buf, 2, HAL_MAX_DELAY);
			ConsoleProcess();
			input_buf_reset(&uart_buf);
			// unsure why console.c doesn't clear, so let's clear it here
			for(uint16_t i=0; i<256; i++) { // should be CONSOLE_COMMAND_MAX_LENGTH, but it's in consoleCommands.h
				mReceiveBuffer[i] = 0;
			}
			mReceivedSoFar = 0;
		}
}
/* USER CODE BEGIN 4 */
// Sonar Calls for each direction
// FUTURE simplify and use different Trigger pins for each
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)  // if the interrupt source is channel1
	{
		if (Is_First_Captured==0) // if the first value is not captured
		{
			IC_Val1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1); // read the first value
			Is_First_Captured = 1;  // set the first captured as true
			// Now change the polarity to falling edge
			__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_FALLING);
		}

		else if (Is_First_Captured==1)   // if the first is already captured
		{
			IC_Val2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);  // read second value
			__HAL_TIM_SET_COUNTER(htim, 0);  // reset the counter

			if (IC_Val2 > IC_Val1)
			{
				Difference = IC_Val2-IC_Val1;
			}

			else if (IC_Val1 > IC_Val2)
			{
				Difference = (0xffff - IC_Val1) + IC_Val2;
			}

			Distance = Difference * .034/2;
			Is_First_Captured = 0; // set it back to false

			// set polarity to rising edge
			__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_RISING);
			__HAL_TIM_DISABLE_IT(&htim1, TIM_IT_CC1);
		}
	}
	if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)  // if the interrupt source is channel1
	{
		if (Is_First_Captured==0) // if the first value is not captured
		{
			IC_Val1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2); // read the first value
			Is_First_Captured = 1;  // set the first captured as true
			// Now change the polarity to falling edge
			__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_2, TIM_INPUTCHANNELPOLARITY_FALLING);
		}

		else if (Is_First_Captured==1)   // if the first is already captured
		{
			IC_Val2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);  // read second value
			__HAL_TIM_SET_COUNTER(htim, 0);  // reset the counter

			if (IC_Val2 > IC_Val1)
			{
				Difference = IC_Val2-IC_Val1;
			}

			else if (IC_Val1 > IC_Val2)
			{
				Difference = (0xffff - IC_Val1) + IC_Val2;
			}

			DistanceR = Difference * .034/2;
			Is_First_Captured = 0; // set it back to false

			// set polarity to rising edge
			__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_2, TIM_INPUTCHANNELPOLARITY_RISING);
			__HAL_TIM_DISABLE_IT(&htim1, TIM_IT_CC2);
		}
	}
	if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3)  // if the interrupt source is channel1
	{
		if (Is_First_Captured==0) // if the first value is not captured
		{
			IC_Val1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_3); // read the first value
			Is_First_Captured = 1;  // set the first captured as true
			// Now change the polarity to falling edge
			__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_3, TIM_INPUTCHANNELPOLARITY_FALLING);
		}

		else if (Is_First_Captured==1)   // if the first is already captured
		{
			IC_Val2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_3);  // read second value
			__HAL_TIM_SET_COUNTER(htim, 0);  // reset the counter

			if (IC_Val2 > IC_Val1)
			{
				Difference = IC_Val2-IC_Val1;
			}

			else if (IC_Val1 > IC_Val2)
			{
				Difference = (0xffff - IC_Val1) + IC_Val2;
			}

			DistanceL = Difference * .034/2;
			Is_First_Captured = 0; // set it back to false

			// set polarity to rising edge
			__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_3, TIM_INPUTCHANNELPOLARITY_RISING);
			__HAL_TIM_DISABLE_IT(&htim1, TIM_IT_CC3);
		}
	}
}
void HCSR04_Read (void)
{
	HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_SET);  // pull the TRIG pin HIGH
	delay_us(10);  // wait for 10 us
	HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_RESET);  // pull the TRIG pin low

	__HAL_TIM_ENABLE_IT(&htim1, TIM_IT_CC1);

}

void HCSR04_ReadRight (void)
{
	HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_SET);  // pull the TRIG pin HIGH
	delay_us(10);  // wait for 10 us
	HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_RESET);  // pull the TRIG pin low

	__HAL_TIM_ENABLE_IT(&htim1, TIM_IT_CC2);


}

void HCSR04_ReadLeft (void)
{
	HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_SET);  // pull the TRIG pin HIGH
	delay_us(10);  // wait for 10 us
	HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_RESET);  // pull the TRIG pin low

	__HAL_TIM_ENABLE_IT(&htim1, TIM_IT_CC3);

}

// Timer for ultrasonic echo
void delay_us(uint16_t us)
{
	__HAL_TIM_SET_COUNTER(&htim2, 0);
	while(__HAL_TIM_GET_COUNTER(&htim2) < us);
}


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	currentMs = HAL_GetTick();
	if((GPIO_Pin == GPIO_PIN_0) && (currentMs - previousMs > 10) ){
		if(HAL_GPIO_ReadPin(GPIOA, GPIO_Pin)){
			// Rising
			HAL_GPIO_WritePin(GPIOD, LD6_Pin, GPIO_PIN_SET);
			button_pressed = true; // exit cli mode
		}
		else{
			// Falling
			HAL_GPIO_WritePin(GPIOD, LD6_Pin, GPIO_PIN_RESET);
		}
	}
	previousMs = currentMs;
}

void straight(uint16_t ms_time)
{
	  TIM3->CCR1 = speed;
	  TIM3->CCR3 = 0;
	  TIM3->CCR2 = speed_R;
	  TIM3->CCR4 = 0;
	  HAL_Delay(ms_time);
	  TIM3->CCR1 = 0;
	  TIM3->CCR3 = 0;
	  TIM3->CCR2 = 0;
	  TIM3->CCR4 = 0;
	  HAL_Delay(1000);
}
void left(uint16_t ms_time)
{
	  TIM3->CCR1 = 0;
	  TIM3->CCR3 = speed;
	  TIM3->CCR2 = speed_R;
	  TIM3->CCR4 = 0;
	  HAL_Delay(ms_time);
	  TIM3->CCR1 = 0;
	  TIM3->CCR3 = 0;
	  TIM3->CCR2 = 0;
	  TIM3->CCR4 = 0;
	  HAL_Delay(1000);
}
void right(uint16_t ms_time)
{
	  TIM3->CCR1 = speed;
	  TIM3->CCR3 = 0;
	  TIM3->CCR2 = 0;
	  TIM3->CCR4 = speed_R;
	  HAL_Delay(ms_time);
	  TIM3->CCR1 = 0;
	  TIM3->CCR3 = 0;
	  TIM3->CCR2 = 0;
	  TIM3->CCR4 = 0;
	  HAL_Delay(1000);
}
void back(uint16_t ms_time)
{
	  TIM3->CCR1 = 0;
	  TIM3->CCR3 = speed;
	  TIM3->CCR2 = 0;
	  TIM3->CCR4 = speed_R;
	  HAL_Delay(ms_time);
	  TIM3->CCR1 = 0;
	  TIM3->CCR3 = 0;
	  TIM3->CCR2 = 0;
	  TIM3->CCR4 = 0;
	  HAL_Delay(1000);
}
void heading_left(void)
{
	if(current_heading == 0)
	{
		current_heading = 3;
	}
	else
	{
		current_heading--;
	}
}
void heading_right(void)
{
	if(current_heading == 3)
	{
		current_heading = 0;
	}
	else
	{
		current_heading++;
	}
}

void wall_follow(void)
{

	  if(DistanceR > wall_space && !block)
	  {
		  // if an open to the right go there
		  right(turn_time);
		  heading_right();
		  straight(forward_time); // unit must go forward or it will circle
		  block = 0;
		  update_heading();
		  op_mode = mode_sensing;
	  }
	  else if(Distance > 30)
	  {
		  //nothing in front
		  straight(forward_time);
		  block = 0;
		  update_heading();
		  op_mode = mode_sensing;

	  }
	  else if(DistanceL > 60)
	  {
		  //if blocked go left
		  left(turn_time);
		  block = 0;
		  heading_left();
	  }
	  else
	  {
		  // dead end back up and don't go straight
		  back(forward_time);
		  block = 1;
		  // update grid location
		  update_heading_back();
	  }

}
void update_heading(void)
{
	switch(current_heading)
	{
	  case north:
		  x_loc++;
		  break;
	  case east:
		  y_loc--;
		  break;
	  case south:
		  x_loc--;
		  break;
	  case west:
		  y_loc++;
		  break;
	}
}
void update_heading_back(void)
{
	  switch(current_heading)
	  {
		  case north:
			  x_loc--;
			  break;
		  case east:
			  y_loc++;
			  break;
		  case south:
			  x_loc++;
			  break;
		  case west:
			  y_loc--;
			  break;
	  }
}

// Check if robot is back to the origin point
void origin_check(void)
{
	if(x_loc == x_ori && y_loc == y_ori)
	{
		if(perimeter_only)
		{
			// Back at the origin then end the session
			op_mode = mode_measure;
		}
		else
		{
			// Move in and go around again
			// Must go in and forward else endless cycle
			// FUTURE to build this for irregular rooms
			left(turn_time);
			heading_left();
			straight(forward_time);
			update_heading();
			right(turn_time);
			heading_right();
			straight(forward_time);
			update_heading();
			// update the loop starting point
			x_ori = x_loc;
			y_ori = y_loc;
			// Set the new distance from the wall
			wall_space = wall_space + 30;
			if(gas_values[x_loc][y_loc != 0])
			{
				// if value has been taken then the grid is closed
				// this only works in rectangular rooms
				// FUTURE change to search grid for zeros
				op_mode = mode_measure;
			}

		}
	}
}

void check_sonar(void)
{
	  HCSR04_Read();
	  HAL_Delay(200);
	  HCSR04_ReadRight();
	  HAL_Delay(200);
	  HCSR04_ReadLeft();
	  HAL_Delay(400);
}

void demo_drive(void)
{
	uint8_t movement_units= 5;
	  for(int i = movement_units; i > 0; i--)
		  {
			  //nothing in front
			  straight(forward_time);
			  block = 0;
			  update_heading();
			  //measure ADC
			  HAL_Delay(1000);
			  gas_values[x_loc][y_loc] = read_gas();
		  }
	  left(turn_time);
	  heading_left();
	  straight(forward_time);
	  left(turn_time);
	  heading_left();
	  gas_values[x_loc][y_loc] = read_gas();
	  for(int i = movement_units; i > 0; i--)
		  {
			  //nothing in front
			  straight(forward_time);
			  block = 0;
			  update_heading();
			  //measure ADC
			  HAL_Delay(1000);
			  gas_values[x_loc][y_loc] = read_gas();
		  }
	linear_reg(2);

}

void findHighest(int* x, int* y)
{
	int n = 10;
	int m = 10;


     for (int i = 0; i < n; i++)
     {
         int max = gas_values[i][0];
         for (int j = 1; j < m; j++)
         {
             if (gas_values[i][j] > max)
             {
                max = gas_values[i][j];
                *x = i;
                *y = j;
             }
         }
     }
     //null the max so we can get the next value
     gas_values[*x][*y] = 0;

}

void linear_reg(int n)
{
	//variables
   int i;
   int x,y;
   float m,c,d;
   float sumx=0,sumxsq=0,sumy=0,sumxy=0;

	//find max points


	// calculate m and c
	for(i=0;i<n;i++){
		findHighest(&y,&x);
	   sumx=sumx+x;
	   sumxsq=sumxsq+(x*x);
	   sumy=sumy+y;
	   sumxy=sumxy+(x*y);
	}

	d=n*sumxsq-sumx*sumx;
	m=(n*sumxy-sumx*sumy)/d;
	c=(sumy*sumxsq-sumx*sumxy)/d;
	uint64_t m_int = (uint64_t)m;
	uint64_t c_int = (uint64_t)c;
	uint64_t flash_data = ((uint64_t)m_int << 32) | c_int;
	flash_write(flash_data);
}

void flash_write(uint64_t data)
{

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
	// FLASH->CR &= (FLASH_CR_PG);

	HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,0x08020000, data);

	HAL_Delay(10);
	HAL_FLASH_Lock();
}

uint16_t read_gas(void)
{
	uint16_t rawADC;
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
	rawADC = HAL_ADC_GetValue(&hadc1);
	return rawADC;
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
