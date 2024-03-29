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
#include <string.h>

//#include "stm32f4xx.h"
#include "stm32f401_gpio.h"
#if !defined(__SOFT_FP__) && defined(__ARM_FP)
  #warning "FPU is not initialized, but the project is compiling for an FPU. Please initialize the FPU before use."
#endif


void delay()
 {
    for (long long int i = 0; i < 500000/2; i++)
    {
        /* code */
    }

 }

int main(void)
{

//     GPIOD_PCLK_EN;
//     GPIOD->MODER |=  (1 << 4)  ;
//     GPIOD->MODER &= ~(1 << 5)  ;

	GPIO_Handler gpio_led;
	    memset(&gpio_led,0,sizeof(gpio_led));
	    gpio_led.GPIOX = GPIOA ;
	    gpio_led.GPIO_Pin_Conf.PinNum   = GPIO_PIN_3;
	    gpio_led.GPIO_Pin_Conf.PinMode  = GPIO_MODE_OUT ;
	    gpio_led.GPIO_Pin_Conf.PinOPTYP = GPIO_OUTYPE_PP ;
	    gpio_led.GPIO_Pin_Conf.PinPUPDR = GPIO_PIN_PUPD_PU ;
	    gpio_led.GPIO_Pin_Conf.PinSpeed = GPIO_OUTSPEED_HIGH ;

	    GPIO_clkCntrl(GPIOA, ENABLE);

        GPIO_init(&gpio_led);


		GPIO_Handler gpio_BT;
	    memset(& gpio_BT,0,sizeof(gpio_BT));

		gpio_BT.GPIOX = GPIOA;
		gpio_BT.GPIO_Pin_Conf.PinNum    = GPIO_PIN_0;
		gpio_BT.GPIO_Pin_Conf.PinMode   = GPIO_MODE_IT_FT;
	    gpio_BT.GPIO_Pin_Conf.PinSpeed  = GPIO_OUTSPEED_HIGH ;
	    gpio_BT.GPIO_Pin_Conf.PinPUPDR  = GPIO_PIN_PUPD_PU ;

	    GPIO_clkCntrl(GPIOA, ENABLE);

        GPIO_init(&gpio_BT);

// irq conf
        GPIO_IRQconf(IRQ_NUM_EXTI0, ENABLE);
        GPIO_PriorityConf(IRQ_PRIRITY_15, IRQ_NUM_EXTI0);

while(1) ;


}

 void EXTI0_IRQHandler (void)
 {
	 delay();
	 GPIO_IRQ_Handle(GPIO_PIN_0);
	 GPIO_ToggPin(GPIOA, GPIO_PIN_3);
 }


