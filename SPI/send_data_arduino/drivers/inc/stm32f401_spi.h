/*
 * stm32f401_spi.h
 *
 *  Created on: Jan 23, 2024
 *      Author: MozaKRA
 */

#ifndef INC_STM32F401_SPI_H_
#define INC_STM32F401_SPI_H_

#include "./stm32f401_gpio.h"


/************************  SPI MACORS  ***********************/
#define NOT_EMPTY             0x00000000U
#define BYTE_1                8
#define BYTE_2                16


/************************  SPI STRUCT  CONFIG   ***********************/

typedef struct
 {
  uint8_t SPI_DeviceMode ;
  uint8_t SPI_BusConfg ;
  uint8_t SPI_DFF ;
  uint8_t SPI_CPHA ;
  uint8_t SPI_CPOL ;
  uint8_t SPI_SSM ;
  uint8_t SPI_Speed ;
 }SPI_Conf;


/************************ SPI STRUCT Handler  ***********************/

 typedef struct
 {
    SPI_REGDEF *pSPI ;
    SPI_Conf SpiConfig;

 }SPI_Handler;


 /*
  * @SPI_DeviceMode
  */
 /************************ SPI DEVICE  MODE Macros  ***********************/
 #define SPI_DEVICE_MODE_MASTER             1
 #define SPI_DEVICE_MODE_SLAVE              0
 /*
  * @SPI_BusConfig
  */
 /************************ SPI BUS CONFIG  MODE Macros  ***********************/
  #define SPI_BUS_CONFIG_FD             1
  #define SPI_BUS_CONFIG_HD             2
  #define SPI_BUS_CONFIG_SIMPLEX_RX     3

 /*
  * @SPI_DFF
  */
 /************************ SPI DATA FORMAT   Macros  ***********************/
  #define SPI_DFF_8_BITS                 0
  #define SPI_DFF_16_BITS                1

 /*
  * @CPHA
  */
 /************************ SPI CPHA Macros  ***********************/
   #define SPI_CPHA_HIGH                1
   #define SPI_CPHA_LOW                 0


 /*
  * @CPOL
  */
 /************************ SPI CPOL Macros  ***********************/
    #define SPI_CPOL_HIGH                1
    #define SPI_CPOL_LOW                 0


 /*
  * @SPI_SSM
  */
 /************************ SPI SOFTWARE SLAVE MANGEMENT   Macros  ***********************/
     #define SPI_SSM_EN                 1
     #define SPI_SSM_DS                 0


 /************************ SPI SPEED   Macros  ***********************/
#define SPI_SPEED_DIV2                 0
#define SPI_SPEED_DIV4                 1U
#define SPI_SPEED_DIV8                 2U
#define SPI_SPEED_DIV16                3U
#define SPI_SPEED_DIV32                4U
#define SPI_SPEED_DIV64                5U
#define SPI_SPEED_DIV128               6U
#define SPI_SPEED_DIV256               7U

 /*
  * SPI related status flags definitions
  */

 /************************ SPI FLAG   Macros  ***********************/

#define SPI_TXE_FLAG                 ( 1 << SPI_SR_TXE  )
#define SPI_RXNE_FLAG                ( 1 << SPI_SR_RXNE )
#define SPI_BSY_FLAG                 ( 1 << SPI_SR_BSY  )




/********************get flag status *************************/

uint8_t GetFlagStatus(SPI_REGDEF *pSPI , uint32_t FlagName);

 /********************clock enable *************************/
void SPI_clkCntrl(SPI_REGDEF *pSpi  , uint8_t EnOrDsi);

/********************SPI INIT *************************/
void SPI_init(SPI_Handler *pSPI_handler);

/********************SPI DEINIT *************************/
void SPI_Deinit(SPI_REGDEF *pSPI);

/********************SPI peripheral control  *************************/
void SPI_ssi_conf(SPI_REGDEF *pSPIx , uint8_t EnOrDsi);


void SPI_PeripheralControl(SPI_REGDEF *pSPIx , uint8_t EnOrDsi);


/********************SPI ssi control  *************************/
void SPI_ssi_conf(SPI_REGDEF *pSPIx , uint8_t EnOrDsi);

void SPI_Disable(SPI_REGDEF *pSPIx);
/********************SPI ssoe control  *************************/
void SPI_ssoe_conf(SPI_REGDEF *pSPIx , uint8_t EnOrDsi);
/********************SPI data send & recieve  *************************/
void SPI_SendData(SPI_REGDEF *pSPIx,uint8_t *pTxBuffer, uint32_t Len);
void SPI_Recieve(SPI_REGDEF *pSPI , uint8_t *pTxBuffer ,uint32_t len);

/********************SPI IRQ CONFIGURATION *************************/
uint8_t SPI_IRQconf (uint8_t IRQnum , uint8_t ENORDS );
uint8_t SPI_PriorityConf ( uint8_t IRQpriority , uint8_t IRQnum);
void SPI_IRQ_Handle (uint8_t pinNum);


#endif /* INC_STM32F401_SPI_H_ */
