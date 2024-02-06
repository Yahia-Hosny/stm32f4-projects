/**
 ******************************************************************************
 * @file           : main.c
 * @author         : Auto-generated by STM32CubeIDE
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */

#include <stdint.h>
//#include "stm32f4xx.h"
#include "stm32f401_gpio.h"
#if !defined(__SOFT_FP__) && defined(__ARM_FP)
  #warning "FPU is not initialized, but the project is compiling for an FPU. Please initialize the FPU before use."
#endif

int main(void)
{

//     GPIOD_PCLK_EN;
//     GPIOD->MODER |=  (1 << 4)  ;
//     GPIOD->MODER &= ~(1 << 5)  ;

	GPIO_Handler gpio_led;
	    gpio_led.GPIOX = GPIOA ;
	    gpio_led.GPIO_Pin_Conf.PinNum   = GPIO_PIN_3;
	    gpio_led.GPIO_Pin_Conf.PinMode  = GPIO_MODE_OUT ;
	    gpio_led.GPIO_Pin_Conf.PinOPTYP = GPIO_OUTYPE_PP ;
	    gpio_led.GPIO_Pin_Conf.PinPUPDR = GPIO_PIN_PUPD_NO ;
	    gpio_led.GPIO_Pin_Conf.PinSpeed = GPIO_OUTSPEED_HIGH ;
GPIO_clkCntrl(GPIOA, ENABLE);

GPIO_init(&gpio_led);

while(1)
{
//	GPIOD->ODR |=  (1 << 2) ;
//	GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_HIGH);
	GPIO_ToggPin(GPIOA, GPIO_PIN_3);
}
}