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
volatile float Kp = 0.0F;
volatile float Kd = 0.0F;
volatile float Ki = 0.0F;

// Define PID variables
volatile float e_n_1 = 0.0F;     // Previous error
volatile float integral = 0.0F;  // Integral term
volatile float Ts = 1.0F / PID_SAMPLING_RATE; // Sampling time


// uint8_t state = IDLE;


// Define PID variables
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

#define NUM_GAINS 3 // Number of gains
typedef enum gain_state {
  GAIN_KP,
  GAIN_KD,
  GAIN_KI
} gain_state_t;

float *gains_array[3] = {&Kp, &Kd, &Ki}; // Array of pointers to gain variables

volatile gain_state_t state = GAIN_KP; // Current gain state

void increment_current_gain()
{
  // Increment the current gain value by 0.1
  *(gains_array[state]) += 0.1f;
}

void change_gain_state()
{
  // Change the current gain state to the next one in the array
  state = (gain_state_t)((state + 1) % NUM_GAINS);
}

// Para ajustar las ganancias, se empleará un botón
// ◼ Si se presiona el botón menos de ¼ de segundo
// se incrementa la ganancia actual en 0.1
// ◼ Si se presiona el botón más de ¾ de segundo, se
// seleccionará la siguiente ganancia a modificar



uint8_t lcd_buffer[33] = {0};  // One extra byte for null terminator

void write_the_stuff()
{
  uint8_t new_buffer[33] = {0}; // One extra byte for null terminator

  // Line 1: " >kp - >kd - >ki"
  // Line 2: "0.00  0.00  0.00"

  uint8_t len = sprintf((char *)new_buffer, " %ckp - %ckd - %cki %d.%d   %d.%d   %d.%d",
    state == 0 ? '>' : ' ',
    state == 1 ? '>' : ' ',
    state == 2 ? '>' : ' ',
    (int)Kp % 10,
    (int)(Kp * 10) % 10,
    (int)Kd % 10,
    (int)(Kd * 10) % 10,
    (int)Ki % 10,
    (int)(Ki * 10) % 10
    );


  for (int i = 0; i < 32; i++) {
    if (new_buffer[i] != lcd_buffer[i]) {
      LCD_Goto_XY(i % 16, i / 16);
      LCD_Write(new_buffer[i] ? new_buffer[i] : ' ', 0);
      lcd_buffer[i] = new_buffer[i];
    }
  }
}

typedef enum {
  BUTTON_IDLE = 0,
  BUTTON_DEBOUNCE,
  BUTTON_PRESSED
} ButtonState_t;
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

  GPIOB->CRH &= ~(0xF << 8); // Clear CNF10 & MODE10 (bits 8-11 for GPIOB10)
  GPIOB->CRH |= (0x8 << 8);  // Set CNF10 as input with pull-up/pull-down (0b1000), MODE10 as input (0b00)

  // Enable pull-up or pull-down resistor
  GPIOB->ODR |= (1 << 10);   // Enable pull-up resistor for GPIOB10

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  LCD_Init(LCD_8B_INTERFACE);

  LCD_Print(" >kp -  kd -  ki"); // Print initial message on LCD
  LCD_Goto_XY(0, 1);
  LCD_Print(" 0.0   0.0   0.0");
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  ButtonState_t buttonState = BUTTON_IDLE;
  uint32_t pressStartTime;
  uint32_t pressDuration = 0;
  while (1)
  {
    // each 100 ms
    //          print "EncPos: {angle value / position in grade}°"
    
    // Check if the button is pressed
    switch (buttonState)
    {
      case BUTTON_IDLE:
        if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_10) == GPIO_PIN_RESET) {
          pressStartTime = HAL_GetTick(); // Record the time when the button is pressed
          buttonState = BUTTON_DEBOUNCE;
        }
        break;
      
      case BUTTON_DEBOUNCE:
        if (HAL_GetTick() - pressStartTime >= 10) {
          if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_10) == GPIO_PIN_RESET) {
            buttonState = BUTTON_PRESSED;
          } else {
            buttonState = BUTTON_IDLE;
            increment_current_gain(); // Example: Increment gain
            write_the_stuff();
          }
        }
        break;
      case BUTTON_PRESSED:
        if (!HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_10) == GPIO_PIN_RESET) {
          buttonState = BUTTON_IDLE;
          pressDuration = HAL_GetTick() - pressStartTime; // Calculate the press duration
          // Add your logic based on the press duration
          if (pressDuration < 250) // Short press (< 250 ms)
          {
              increment_current_gain(); // Example: Increment gain
              write_the_stuff();
          }
          else// if (pressDuration >= 750) // Long press (>= 750 ms)
          {
              change_gain_state(); // Example: Change gain state
              write_the_stuff();
          }
        }
        break;
      default:
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
