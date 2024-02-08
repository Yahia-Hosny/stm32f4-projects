/*
 * stm32f401_gpio.h
 *
 *  Created on: Jan 3, 2024
 *      Author: MozaKRA
 */

#ifndef STM32F401_GPIO_H_
#define STM32F401_GPIO_H_



#endif /* STM32F401_GPIO_H_ */


#include "stm32f4xx.h"


/************************  GPIO Driver  MODE Macros  ***********************/
#define GPIO_MODE_IN                          0
#define GPIO_MODE_OUT                         1
#define GPIO_MODE_ALTFN                       2
#define GPIO_MODE_ANALOG                      3
#define GPIO_MODE_IT_FT                       4
#define GPIO_MODE_IT_RT                       5
#define GPIO_MODE_IT_RFT                      6

/************************  GPIO Driver OUTPUT TYPE Macros  ***********************/
#define GPIO_OUTYPE_PP                        0
#define GPIO_OUTYPE_OD                        1


/************************  GPIO Driver OUTPUT SPEED Macros  ***********************/
#define GPIO_PINSPEED_LOW                     0
#define GPIO_PINSPEED_MED                     1
#define GPIO_PINSPEED_HIGH                    2
#define GPIO_PINSPEED_VERYHIGH                3



/************************  GPIO Driver PULL UP PULL DOWN CONTROL  Macros  ***********************/
#define GPIO_PUPD_NO                      0
#define GPIO_PUPD_PU                      1
#define GPIO_PUPD_PD                      2


/************************  PIN macros  ***********************/

#define GPIO_PIN_0              0
#define GPIO_PIN_1              1
#define GPIO_PIN_2              2
#define GPIO_PIN_3              3
#define GPIO_PIN_4              4
#define GPIO_PIN_5              5
#define GPIO_PIN_6              6
#define GPIO_PIN_7              7
#define GPIO_PIN_8              8
#define GPIO_PIN_9              9
#define GPIO_PIN_10             10
#define GPIO_PIN_11             11
#define GPIO_PIN_12             12
#define GPIO_PIN_13             13
#define GPIO_PIN_14             14
#define GPIO_PIN_15             15

#define GPIO_PINNumbers         15

/************************  GPIO STRUCT PIN CONFIG   ***********************/

 typedef struct
 {
  uint8_t PinNum ;
  uint8_t PinMode ;
  uint8_t PinSpeed ;
  uint8_t PinPUPDR ;
  uint8_t PinOPTYP ;
  uint8_t PinAltFn ;
 }GPIO_Pin_Conf;


/************************  GPIO STRUCT Handler  ***********************/

 typedef struct
 {
   GPIO_REGDEF    *GPIOX ;
   GPIO_Pin_Conf  GPIO_Pin_Conf ;

 }   GPIO_Handler ;



/********************clock enable *************************/
void GPIO_clkCntrl(GPIO_REGDEF *pGpio  , uint8_t EnOrDsi);





/********************GPIO INIT *************************/
void GPIO_init(GPIO_Handler *pGpio);

/********************GPIO DEINIT *************************/
void GPIO_Deinit(GPIO_REGDEF *pGpio);

/********************GPIO Read Pin *************************/
uint8_t GPIO_ReadPin(GPIO_REGDEF *pGpiox ,uint8_t pinNum);

/********************GPIO Read port *************************/
uint16_t GPIO_ReadPort(GPIO_REGDEF *pGpiox );



/********************GPIO write pin *************************/
void GPIO_WritePin(GPIO_REGDEF *pGpiox  , uint8_t pinNum ,uint8_t value );

/********************GPIO write port *************************/
void  GPIO_WritePort(GPIO_REGDEF *pGpiox , uint16_t value );

/********************GPIO toggle pin *************************/
void GPIO_ToggPin(GPIO_REGDEF *pGpiox  , uint8_t pinNum  );

/********************GPIO toggle port *************************/
void  GPIO_ToggPort(GPIO_REGDEF *pGpiox );


/********************GPIO IRQ CONFIGURATION *************************/
void GPIO_IRQconf (uint8_t IRQnum , uint8_t ENORDS );
void GPIO_PriorityConf ( uint8_t IRQpriority , uint8_t IRQnum);
void GPIO_IRQ_Handle (uint8_t pinNum);
