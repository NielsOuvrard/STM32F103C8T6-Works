/*
 * common.h
 *
 *  Created on: Feb 24, 2025
 *      Author: User123
 */

 #ifndef COMMON_H_
 #define COMMON_H_

 /* Exported includes -----------------------------------------------*/
 #include <stdint.h>
 #include <string.h>

 /* Exported defines ------------------------------------------------*/
 #define SystemCoreClock 8000000UL

 /* Exported macros -------------------------------------------------*/
 #define BIT(n)				(1UL << (n))

 #define __enable_irq()	__asm volatile ("cpsie i" : : : "memory")
 #define __disable_irq() __asm volatile ("cpsid i" : : : "memory")

 #define __NOP() __asm volatile ("nop")

 /* Exported typedefs ------------------------------------------------*/
 typedef enum
 {
     STM32_FAIL = -1,
     STM32_OK = 0,
     STM32_ERR_INVALID_ARG,
     STM32_ERR_NOT_FOUND,
     STM32_ERR_TIMEOUT,
     STM32_ERR_OUT_RANGE,
     STM32_ERR_NULL_POINTER
 } stm32_err_t;

 /* Exported variables -----------------------------------------------*/

 /* Exported prototype function --------------------------------------*/

 /* Exported inline function --------------------------------------*/
 void delay_ms(uint32_t ms);
 void delay_10us(uint32_t usx10);
 void delay_us(uint32_t us);

 #endif /* COMMON_H_ */
