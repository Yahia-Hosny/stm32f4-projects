/*
 * stm32f4xx.h
 *
 *  Created on: Jan 3, 2024
 *      Author: MozaKRA
 */

#ifndef STM32F4XX_H_
#define STM32F4XX_H_



#endif /* STM32F4XX_H_ */

#include <stdint.h>

#define __vo   				volatile
/************************  processor specific  macros  ***********************/
/*
* nvic enable interrupt set
*/
#define NVIC_ISER0               ( * (__vo uint32_t *)(0xE000E100) )
#define NVIC_ISER1               ( * (__vo uint32_t *)(0xE000E104) )
#define NVIC_ISER2               ( * (__vo uint32_t *)(0xE000E108) )
#define NVIC_ISER3               ( * (__vo uint32_t *)(0xE000E10C) )


/*
* nvic enable interrupt clear
*/
#define NVIC_ICER0               ( * (__vo uint32_t *)(0XE000E180) )
#define NVIC_ICER1               ( * (__vo uint32_t *)(0XE000E184) )
#define NVIC_ICER2               ( * (__vo uint32_t *)(0XE000E188) )
#define NVIC_ICER3               ( * (__vo uint32_t *)(0XE000E18C) )

/*interrupt priority */
#define NVIC_IPR                 (  (__vo uint32_t *)(0xE000E400) )

#define NUM_PR_BITS_IMPLEMENTED        4


/* PRIORITY NUMBER MACROS */
#define IRQ_PRIRITY_0               0
#define IRQ_PRIRITY_1               1
#define IRQ_PRIRITY_2               2
#define IRQ_PRIRITY_3               3
#define IRQ_PRIRITY_4               4
#define IRQ_PRIRITY_5               5
#define IRQ_PRIRITY_6               6
#define IRQ_PRIRITY_7               7
#define IRQ_PRIRITY_8               8
#define IRQ_PRIRITY_9               9
#define IRQ_PRIRITY_10              10
#define IRQ_PRIRITY_11              11
#define IRQ_PRIRITY_12              12
#define IRQ_PRIRITY_13              13
#define IRQ_PRIRITY_14              14
#define IRQ_PRIRITY_15              15


/************************  some generic macros  ***********************/

#define ENABLE 				1
#define DISABLE 			0
#define SET 				ENABLE
#define RESET 				DISABLE
#define GPIO_PIN_SET        SET
#define GPIO_PIN_RESET      RESET
#define FLAG_RESET          RESET
#define FLAG_SET 			SET

/************************  bit MANP macros  ***********************/

#define SET_BIT(var, bit)       ((var) |= (1 << (bit)))       // Macro to set a specific bit in a variable
#define CLR_BIT(var, bit)       ((var) &= ~(1 << (bit)))     // Macro to reset a specific bit in a variable
#define READ_BIT(var, bit)      (((var) >> (bit)) & 1)      // Macro to read a specific bit from a variable
#define TOGG_BIT(var, bit)      ((var) ^= (1 << (bit)))    // Macro to toggle a specific bit in a variable

/************************  bit macros  ***********************/

#define BIT_0              0
#define BIT_1              1
#define BIT_2              2
#define BIT_3              3
#define BIT_4              4
#define BIT_5              5
#define BIT_6              6
#define BIT_7              7
#define BIT_8              8
#define BIT_9              9
#define BIT_10             10
#define BIT_11             11
#define BIT_12             12
#define BIT_13             13
#define BIT_14             14
#define BIT_15             15
#define BIT_16             16
#define BIT_17             17
#define BIT_18             18
#define BIT_19             19
#define BIT_20             20
#define BIT_21             21
#define BIT_22             22
#define BIT_23             23
#define BIT_24             24
#define BIT_25             25
#define BIT_26             26
#define BIT_27             27
#define BIT_28             28
#define BIT_29             29
#define BIT_30             30
#define BIT_31             31


#define BIT_MASK_8         0x00000001



/************************  MACROS FOR THE MEMORY  ***********************/

#define FLASH_BASEADDR     0x08000000U
#define SRAM_BASEADDR      0x20000000U
#define SRAM               SRAM_BASEADDR
#define ROM_BASEADDR       0x1FFF0000U


/************************  MACROS FOR THE BUSEES  ***********************/

#define PERIPH_BASEADDR    0x40000000U
#define APB1_BASEADDR      0x40000000U
#define APB2_BASEADDR      0x40010000U
#define AHB1_BASEADDR      0x40020000U
#define AHB2_BASEADDR      0x50000000U

/************************  MACROS FOR THE AHB1 BUS  ***********************/

#define GPIOA_BASEADDR     (AHB1_BASEADDR+0X0000)
#define GPIOB_BASEADDR     (AHB1_BASEADDR+0X0400)
#define GPIOC_BASEADDR     (AHB1_BASEADDR+0X0800)
#define GPIOD_BASEADDR     (AHB1_BASEADDR+0X0C00)

#define RCC_BASEADDR       (AHB1_BASEADDR +0x3800)

/************************  MACROS FOR THE APB1 BUS  ***********************/

#define I2C1_BASEADDR     (APB1_BASEADDR+0X5400)
#define I2C2_BASEADDR     (APB1_BASEADDR+0X5800)
#define I2C3_BASEADDR     (APB1_BASEADDR+0X5C00)
#define SPI2_BASEADDR     (APB1_BASEADDR+0X3800)
#define SPI3_BASEADDR     (APB1_BASEADDR+0X3C00)
#define USART2_BASEADDR   (APB1_BASEADDR+0X4400)


/************************  MACROS FOR THE APB2 BUS  ***********************/

#define EXTI_BASEADDR     (APB2_BASEADDR+0X3C00)
#define SYSCFG_BASEADDR   (APB2_BASEADDR+0X3800)
#define SPI1_BASEADDR     (APB2_BASEADDR+0X3000)
#define SPI4_BASEADDR     (APB2_BASEADDR+0X3400)
#define USART1_BASEADDR   (APB2_BASEADDR+0X1000)
#define USART6_BASEADDR   (APB2_BASEADDR+0X1400)


 /************************  GPIO STRUCT REGISTER DEFINATION  ***********************/

 typedef struct
 {
  __vo uint32_t MODER ;
  __vo uint32_t OTYPER ;
  __vo uint32_t OSPEEDR ;
  __vo uint32_t PUPDR ;
  __vo uint32_t IDR ;
  __vo uint32_t ODR ;
  __vo uint32_t BSRR ;
  __vo uint32_t LCKR ;
  __vo uint32_t AFR[2] ;
 } GPIO_REGDEF;



 /************************  RCC STRUCT REGISTER DEFINATION  ***********************/

 typedef struct
 {
  __vo uint32_t CR ;
  __vo uint32_t PLLCFGR ;
  __vo uint32_t CFGR ;
  __vo uint32_t CIR ;
  __vo uint32_t AHB1RSTR ;
  __vo uint32_t AHB2RSTR ;
  __vo uint32_t RESERVED[2] ;
  __vo uint32_t APB1RSTR ;
  __vo uint32_t APB2RSTR ;
  __vo uint32_t RESERVED2[2] ;
  __vo uint32_t AHB1ENR ;
  __vo uint32_t AHB2ENR ;
  __vo uint32_t RESERVED3[2] ;
  __vo uint32_t APB1ENR ;
  __vo uint32_t APB2ENR ;
  __vo uint32_t RESERVED4[2] ;
  __vo uint32_t AHB1LPENR ;
  __vo uint32_t AHB2LPENR ;
  __vo uint32_t RESERVED5[2] ;
  __vo uint32_t APB1LPENR ;
  __vo uint32_t APB2LPENR ;
  __vo uint32_t RESERVED6[2] ;
  __vo uint32_t BDCR ;
  __vo uint32_t CSR ;
  __vo uint32_t RESERVED7[2] ;
  __vo uint32_t SSCGR ;
  __vo uint32_t PLLI2SCFGR ;
  __vo uint32_t RESERVED8 ;
  __vo uint32_t DCKCFGR ;
 } RCC_REGDEF;


 /************************  EXTI STRUCT REGISTER DEFINATION  ***********************/

  typedef struct
  {
   __vo uint32_t IMR ;
   __vo uint32_t EMR ;
   __vo uint32_t RTSR ;
   __vo uint32_t FTSR ;
   __vo uint32_t SWIER ;
   __vo uint32_t PR ;
  } EXTI_REGDEF;


 /************************  SYSCFG STRUCT REGISTER DEFINATION  ***********************/

  typedef struct
  {
   __vo uint32_t MEMRMP ;
   __vo uint32_t PMC ;
   __vo uint32_t EXTICR[4] ;
   __vo uint32_t RESERVED[2] ;
   __vo uint32_t CMPCR ;
  } SYSCFG_REGDEF;

 /*@GPIO_MACROS*/
 /************************  GPIO TYPE CAST ***********************/

#define GPIOA 				((GPIO_REGDEF*)(GPIOA_BASEADDR))
#define GPIOB 				((GPIO_REGDEF*)(GPIOB_BASEADDR))
#define GPIOC 				((GPIO_REGDEF*)(GPIOC_BASEADDR))
#define GPIOD 				((GPIO_REGDEF*)(GPIOD_BASEADDR))


 /************************  RCC TYPE CAST ***********************/

#define RCC                 ((RCC_REGDEF *)(RCC_BASEADDR))

  /************************  EXTI TYPE CAST ***********************/

  #define EXTI                ((EXTI_REGDEF *)(EXTI_BASEADDR))

  /************************  SYSCFG TYPE CAST ***********************/

  #define SYSCFG               ((SYSCFG_REGDEF *)(SYSCFG_BASEADDR))

/**********
 * ******************
 * *********************
 * **************************/


 /************************  GPIO CLOCK ENABLE MACROS  ***********************/

#define GPIOA_PCLK_EN        (SET_BIT((RCC -> AHB1ENR),(BIT_0)))
#define GPIOB_PCLK_EN        (SET_BIT((RCC -> AHB1ENR),(BIT_1)))
#define GPIOC_PCLK_EN        (SET_BIT((RCC -> AHB1ENR),(BIT_2)))
#define GPIOD_PCLK_EN        (SET_BIT((RCC -> AHB1ENR),(BIT_3)))

 /************************  I2C CLOCK ENABLE MACROS  ***********************/
#define I2C1_PCLK_EN        (SET_BIT((RCC -> APB1ENR),(BIT_21)))
#define I2C2_PCLK_EN        (SET_BIT((RCC -> APB1ENR),(BIT_22)))
#define I2C3_PCLK_EN        (SET_BIT((RCC -> APB1ENR),(BIT_23)))

 /************************  I2C CLOCK ENABLE MACROS  ***********************/
#define SPI1_PCLK_EN        (SET_BIT((RCC -> APB2ENR),(BIT_12)))
#define SPI2_PCLK_EN        (SET_BIT((RCC -> APB1ENR),(BIT_14)))
#define SPI3_PCLK_EN        (SET_BIT((RCC -> APB1ENR),(BIT_15)))
#define SPI4_PCLK_EN        (SET_BIT((RCC -> APB2ENR),(BIT_13)))

 /************************  USART CLOCK ENABLE MACROS  ***********************/
#define USART1_PCLK_EN      (SET_BIT((RCC -> APB1ENR),(BIT_4)))
#define USART2_PCLK_EN      (SET_BIT((RCC -> APB2ENR),(BIT_17)))
#define USART6_PCLK_EN      (SET_BIT((RCC -> APB1ENR),(BIT_5)))

 /************************  SYSCONFG CLOCK ENABLE MACROS  ***********************/
#define SYSCFG_PCLK_EN      (SET_BIT((RCC -> APB2ENR),(BIT_14)))


/************************  GPIO CLOCK RESET MACROS  ***********************/

#define GPIOA_RESET       do{(SET_BIT((RCC -> AHB1RSTR),(BIT_0))); CLR_BIT((RCC -> AHB1RSTR),(BIT_0)) ;} while (0)
#define GPIOB_RESET       do{(SET_BIT((RCC -> AHB1RSTR),(BIT_1))); CLR_BIT((RCC -> AHB1RSTR),(BIT_1)) ;} while (0)
#define GPIOC_RESET       do{(SET_BIT((RCC -> AHB1RSTR),(BIT_2))); CLR_BIT((RCC -> AHB1RSTR),(BIT_2)) ;} while (0)
#define GPIOD_RESET       do{(SET_BIT((RCC -> AHB1RSTR),(BIT_3))); CLR_BIT((RCC -> AHB1RSTR),(BIT_3)) ;} while (0)





/**********
 * ******************
 * *********************
 * **************************/



/************************  GPIO CLOCK DISABLE MACROS  ***********************/

#define GPIOA_PCLK_DS        (CLR_BIT((RCC -> AHB1ENR),(BIT_0)))
#define GPIOB_PCLK_DS        (CLR_BIT((RCC -> AHB1ENR),(BIT_1)))
#define GPIOC_PCLK_DS        (CLR_BIT((RCC -> AHB1ENR),(BIT_2)))
#define GPIOD_PCLK_DS        (CLR_BIT((RCC -> AHB1ENR),(BIT_3)))

 /************************  I2C CLOCK DISABLE MACROS  ***********************/
#define I2C1_PCLK_DS        (CLR_BIT((RCC -> APB1ENR),(BIT_21)))
#define I2C2_PCLK_DS        (CLR_BIT((RCC -> APB1ENR),(BIT_22)))
#define I2C3_PCLK_DS        (CLR_BIT((RCC -> APB1ENR),(BIT_23)))

 /************************  I2C CLOCK DISABLE MACROS  ***********************/
#define SPI1_PCLK_DS        (CLR_BIT((RCC -> APB2ENR),(BIT_12)))
#define SPI2_PCLK_DS        (CLR_BIT((RCC -> APB1ENR),(BIT_14)))
#define SPI3_PCLK_DS        (CLR_BIT((RCC -> APB1ENR),(BIT_15)))
#define SPI4_PCLK_DS        (CLR_BIT((RCC -> APB2ENR),(BIT_13)))

 /************************  USART CLOCK DISABLE MACROS  ***********************/
#define USART1_PCLK_DS      (CLR_BIT((RCC -> APB1ENR),(BIT_4)))
#define USART2_PCLK_DS      (CLR_BIT((RCC -> APB2ENR),(BIT_17)))
#define USART6_PCLK_DS      (CLR_BIT((RCC -> APB1ENR),(BIT_5)))

 /************************  SYSCONFG CLOCK DISABLE MACROS  ***********************/
#define SYSCFG_PCLK_DS      (CLR_BIT((RCC -> APB2ENR),(BIT_14)))

 /************************  GPIO_BASE_CODE MACROS  ***********************/
 #define PA              0
 #define PB              1
 #define PC              2
 #define PD              3

 #define GPIO_BASE_CODE(x)      ((x == GPIOA) ? PA :\
                                 (x == GPIOB) ? PB :\
                                 (x == GPIOC) ? PC :\
                                 (x == GPIOD) ? PD :0)


 /************************  GPIO_EXTI IRQ MACROS  ***********************/

 #define IRQ_NUM_EXTI0        6
 #define IRQ_NUM_EXTI1        7
 #define IRQ_NUM_EXTI2        8
 #define IRQ_NUM_EXTI3        9
 #define IRQ_NUM_EXTI4        10
 #define IRQ_NUM_EXTI9_5      23
 #define IRQ_NUM_EXTI10_15    40

