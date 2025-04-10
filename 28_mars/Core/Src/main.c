/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "tim.h"
#include "lcd.h"
#include "gpio.h"
#include <string.h>
#include <stdio.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ENCODER_PPR			(334)
#define PID_SAMPLING_RATE	(100)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/*
 * Eight-bit encoder state register
 * ----------------------------------------------
 * |	B7	B6	B5	B4	|	B3	B2	|	B1	B0	|
 * ----------------------------------------------
 * |	RESERVED		| NextState	| CurState	|
 * |					|-----------|-----------|
 * |					|ENCB* ENCA*|ENCB	ENCA|
 * |---------------------------------------------
 */
volatile uint8_t EncState = 0; // Estado del encoder
volatile int32_t EncPulseCounter = 0;
volatile int32_t lEncCount = 0; // Q[n]
volatile int32_t lEncPrevCount = 0; // Q[n-1]

//  SetPoint
#define SPEED 16000
#define MOTOR_VOLTAGE 9 // 9Volts
#define MOTOR_MAX_VOLTAGE 24
#define SET_POINT ((SPEED / MOTOR_MAX_VOLTAGE) / MOTOR_VOLTAGE)

#define PID_MAX_OUTPUT 1000.0F // Max PWM value
#define PID_MIN_OUTPUT 0.0F // Min PWM value

// Define PID gains
volatile float Kp = 0.2F;
volatile float Kd = 0.1F;
volatile float Ki = 0.0F;

// Define PID variables
volatile float e_n_1 = 0.0F;     // Previous error
volatile float integral = 0.0F;  // Integral term
volatile float Ts = 1.0F / PID_SAMPLING_RATE; // Sampling time


// typedef enum {
// 	IDLE,
// 	BUTTON_PRESSED
// } ButtonState_t;

// uint8_t state = IDLE;


// Define PID variables
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

volatile uint32_t pwm_control = 0; // PWM control signal

#define VALUE_ACTUAL 0
#define VALUE_LAST 1
#define DELTA_TIME 0.01


int32_t get_speed_motor(int32_t q_n, int32_t q_n_1)
{
	return (q_n - q_n_1) / (DELTA_TIME * ENCODER_PPR);
}

uint8_t lcd_buffer[32] = {0};

void write_the_stuff(float angle_value)
{
  uint8_t new_buffer[32] = {0};

  angle_value = 0.23;
  uint8_t len = sprintf((char *)new_buffer, "EncPos: %.2f", angle_value); // ? angle value / position in grade
  new_buffer[len + 1] = 0xDF; // Write degree symbol on LCD

  for (int i = 0; i < 16; i++) {
    if (new_buffer[i] != lcd_buffer[i]) {
      LCD_Goto_XY(i, 0);
      LCD_Write(new_buffer[i] ? new_buffer[i] : ' ', 0);
      lcd_buffer[i] = new_buffer[i];
    }
  }
}

float PID_algorithm(float e_n)
{
	// Apply PID algorithm with x[n] = current angular speed
	// u[n] = Kp*e[n] + Ki*(ErrorSum[n-1] + e[n])*Ts + Kd*(e[n] - e[n-1])/Ts
	// Where e[n] = SetPoint - x[n]
	// Ts: PID Sampling Period
	float derivative = (e_n - e_n_1) / Ts;

	return Kp * e_n + Ki * (integral + e_n) * Ts + Kd * derivative;
}

void EXTI15_10_IRQHandler(void)
{
	if(EXTI->PR & EXTI_PR_PIF10 || EXTI->PR & EXTI_PR_PIF11)
	{
		// Clear EXTI10 & EXTI11 flags by writing 1
		EXTI->PR = EXTI_PR_PIF10 | EXTI_PR_PIF11;

		// Compute next state
		uint8_t nextState = EncState & 0x03;	// Clear next state bit-field
		if (ENCA_GPIO_Port->IDR & ENCA_Pin) nextState |= (1 << 2);
		if (ENCA_GPIO_Port->IDR & ENCB_Pin) nextState |= (1 << 3);
		
		// Update the pulse counter
		/*                        _______         _______
		 *               A ______|       |_______|       |______ A
		 * negative <---      _______         _______         __      --> positive
		 *               B __|       |_______|       |_______|   B
		 *
		 *
		 * |	HEX	|	B*	A*	|	B	A	|	Operation
		 * |--------|-----------|-----------|-------------
		 * |  0x00	|	0	0	|	0	0	|	IDLE
		 * |  0x01	|	0	0	|	0	1	|	+1
		 * |  0x02	|	0	0	|	1	0	|	-1
		 * |  0x03	|	0	0	|	1	1	|   +2
		 * |  0x04	|	0	1	|	0	0	|	-1
		 * |  0x05	|	0	1	|	0	1	|	IDLE
		 * |  0x06	|	0	1	|	1	0	|	-2
		 * |  0x07	|	0	1	|	1	1	|	+1
		 * |  0x08	|	1	0	|	0	0	|	+1
		 * |  0x09	|	1	0	|	0	1	|	-2
		 * |  0x0A	|	1	0	|	1	0	|	IDLE
		 * |  0x0B	|	1	0	|	1	1	|	-1
		 * |  0x0C	|	1	1	|	0	0	|	+2
		 * |  0x0D	|	1	1	|	0	1	|	-1
		 * |  0x0E	|	1	1	|	1	0	|	+1
		 * |  0x0F	|	1	1	|	1	1	|	IDLE
		 * |--------|-----------|-----------|-------------
		 */
		switch(nextState)
		{
			case 0x01: case 0x07: case 0x08: case 0x0E:
				EncPulseCounter++;
			break;

			case 0x02: case 0x04: case 0x0B: case 0x0D:
				EncPulseCounter--;
			break;

			case 0x03: case 0x0C:
				EncPulseCounter += 2;
			break;

			case 0x06: case 0x09:
				EncPulseCounter -= 2;
			break;
		}

		// Update current state with next state
		EncState = nextState >> 2;
	}
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
	RCC->APB2ENR |= (0x01 << 4); // Enable GPIOC clock
  RCC->APB2ENR |= (0x01 << 3); // Enable GPIOB clock
	GPIOC->CRH &= ~(0xF << 20); // Clear CNF13 & MODE13
  GPIOC->CRH |= (0x01 << 20); // output mode, max speed 10 MHz
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  LCD_Init(LCD_8B_INTERFACE);

  sprintf(lcd_buffer, "EncPos: 0"); // Initialize lcd buffer with initial message
  LCD_Print(lcd_buffer); // Print initial message on LCD
  LCD_Write(0xDF, 0); // Write degree symbol on LCD
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM4_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  // Initialize PID variables
  
  HAL_NVIC_SetPriority(TIM4_IRQn, 1, 0); // ? which priority should be at 1 ? I put both just in case

  // Set current encoder state
  if (ENCA_GPIO_Port->IDR & ENCA_Pin) EncState |= 0x01;
  if (ENCA_GPIO_Port->IDR & ENCB_Pin) EncState |= 0x02;
  
  // Hardware
  SysTick_Config(SystemCoreClock / 1000); 		// Systick to 1ms
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);		// Start TIM4.PWM.CH1

  // Enable EXTI15_10 Interrupts
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  uint32_t ulLcdPrintTime = HAL_GetTick(); // lcd stuff I guess
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  uint32_t ulPIDLastTimeWake = HAL_GetTick(); // Get current time in milliseconds
  lEncPrevCount = 0; // q[n - 1]
  while (1)
  {
    // each 100 ms
    //          print "EncPos: {angle value / position in grade}°"
    if (HAL_GetTick() - ulLcdPrintTime >= 2000)
    {
      // Get current time
      ulLcdPrintTime = HAL_GetTick();
      
      // Print the encoder position in degrees
      float fEncPos = (float)EncPulseCounter * 360.0F / ENCODER_PPR;
      write_the_stuff(fEncPos);
    }
    
    // each 1ms
    //          algoritmo PID para el control de la posición del motor
	  // Check if PID algorithm must be applied
	  if((HAL_GetTick() - ulPIDLastTimeWake) >= 1)
	  {
		  // Get current time
		  ulPIDLastTimeWake = HAL_GetTick();

		  // Get the current encoder pulse counter
		  __disable_irq();
		  lEncCount = EncPulseCounter; // q[n]
		  __enable_irq();
		  
		  // Compute angular speed (RPMs) x[n] = ([CurrentCount - PrevCount]/Ts) * 60 / PPR
		  float fMotorSpeed = get_speed_motor(lEncCount, lEncPrevCount);
		  
		  // Update previous encoder pulse counter with the current encoder pulse counter
		  lEncPrevCount = lEncCount;

		  // Apply PID algorithm with x[n] = current angular speed
		  // u[n] = Kp*e[n] + Ki*(ErrorSum[n-1] + e[n])*Ts + Kd*(e[n] - e[n-1])/Ts
		  // Where e[n] = SetPoint - x[n]
		  // Ts: PID Sampling Period
		  float e_n = SET_POINT - fMotorSpeed;
		  float fPwmControl = PID_algorithm(e_n);
		  
		  e_n_1 = e_n;
		  integral += e_n;

		  // Saturate the output
		  if(fPwmControl > PID_MAX_OUTPUT) fPwmControl = PID_MAX_OUTPUT;
		  else if(fPwmControl < PID_MIN_OUTPUT) fPwmControl = PID_MIN_OUTPUT;
		  
		  // Change the motor direction if needed based on the sign of the fPwmControl
		  if(fPwmControl < 0.0F)
		  {
        // Set motor backward direction on L298N
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET); // IN1 = 0
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);   // IN2 = 1
			  
			  // Change to positive
			  fPwmControl = -fPwmControl;
		  }
		  else
		  {
			  // Set motor forward direction on L298N
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);     // IN1 = 0
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);   // IN2 = 1
		  }

		  // Update duty cycle
		  HAL_PWM_SetDuty((uint32_t)fPwmControl);
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_PWM_SetDuty(uint32_t dc)
{
	if(dc > TIM4->ARR) TIM4->CCR1 = TIM4->ARR;
	else TIM4->CCR1 = dc;
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
