

#include<string.h>
#include "stm32f401_spi.h"

void delay(void)
{
	for(uint32_t i = 0 ; i < 500000 ; i ++);
}

/*
 * PB14 --> SPI2_MISO
 * PB15 --> SPI2_MOSI
 * PB13 -> SPI2_SCLK
 * PB12 --> SPI2_NSS
 * ALT function mode : 5
 */

/* SPI Slave Demo

 *
 * SPI pin numbers:
 * SCK   13  // Serial Clock.
 * MISO  12  // Master In Slave Out.
 * MOSI  11  // Master Out Slave In.
 * SS    10  // Slave Select . Arduino SPI pins respond only if SS pulled low by the master
 *

 */

void SPI2_GPIOInits(void)
{
	GPIO_Handler SPIPins;

	SPIPins.GPIOX = GPIOB;
	SPIPins.GPIO_Pin_Conf.PinMode  = GPIO_MODE_ALTFN;
	SPIPins.GPIO_Pin_Conf.PinAltFn = 5;
	SPIPins.GPIO_Pin_Conf.PinOPTYP = GPIO_OUTYPE_PP;
	SPIPins.GPIO_Pin_Conf.PinPUPDR = GPIO_PIN_PUPD_NO;
	SPIPins.GPIO_Pin_Conf.PinSpeed = GPIO_OUTSPEED_HIGH;

	//SCLK
	SPIPins.GPIO_Pin_Conf.PinNum = GPIO_PIN_13;
	GPIO_init(&SPIPins);

	//MOSI
	SPIPins.GPIO_Pin_Conf.PinNum  = GPIO_PIN_15;
	GPIO_init(&SPIPins);
	//MISO
	//SPIPins.GPIO_Pin_Conf.GPIO_PinNumber = GPIO_PIN_NO_14;
	//GPIO_init(&SPIPins);


	//NSS
	SPIPins.GPIO_Pin_Conf.PinNum = GPIO_PIN_12;
	GPIO_init(&SPIPins);

}

void SPI2_Inits(void)
{

	SPI_Handler SPI2handle;

	SPI2handle.pSPI = SPI2;
	SPI2handle.SpiConfig.SPI_BusConfg = SPI_BUS_CONFIG_FD;
	SPI2handle.SpiConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	SPI2handle.SpiConfig.SPI_Speed = SPI_SPEED_DIV32;
	SPI2handle.SpiConfig.SPI_DFF = SPI_DFF_8_BITS;
	SPI2handle.SpiConfig.SPI_CPOL = SPI_CPOL_LOW;
	SPI2handle.SpiConfig.SPI_CPHA = SPI_CPHA_LOW;
	SPI2handle.SpiConfig.SPI_SSM = SPI_SSM_DS; //Hardware slave management enabled for NSS pin

	SPI_init(&SPI2handle);
}

void GPIO_ButtonInit(void)
{
	GPIO_Handler GPIOBtn;

	//this is btn gpio configuration
	GPIOBtn.GPIOX = GPIOA;
	GPIOBtn.GPIO_Pin_Conf.PinNum    = GPIO_PIN_0;
	GPIOBtn.GPIO_Pin_Conf.PinMode   = GPIO_MODE_IN;
	GPIOBtn.GPIO_Pin_Conf.PinSpeed  = GPIO_OUTSPEED_HIGH;
	GPIOBtn.GPIO_Pin_Conf.PinPUPDR  = GPIO_PIN_PUPD_NO;

	GPIO_init(&GPIOBtn);

}


int main(void)
{
	char user_data[] = "An Arduino Uno board is best suited for beginners who have just started using microcontrollers, on the other hand, Arduino Mega board is for enthusiasts who require a lot of I/O pins for their projects";

	GPIO_ButtonInit();

	//this function is used to initialize the GPIO pins to behave as SPI2 pins
	SPI2_GPIOInits();

	//This function is used to initialize the SPI2 peripheral parameters
	SPI2_Inits();

	/*
	* making SSOE 1 does NSS output enable.
	* The NSS pin is automatically managed by the hardware.
	* i.e when SPE=1 , NSS will be pulled to low
	* and NSS pin will be high when SPE=0
	*/
	SPI_ssoe_conf(SPI2,ENABLE);

	while(1)
	{
		//wait till button is pressed
		while( ! GPIO_ReadPin(GPIOA,GPIO_PIN_0) );

		//to avoid button de-bouncing related issues 200ms of delay
		delay();

		//enable the SPI2 peripheral
		SPI_PeripheralControl(SPI2,ENABLE);

		//first send length information
		uint8_t dataLen = strlen(user_data);
		SPI_SendData(SPI2,&dataLen,1);

		//to send data
		SPI_SendData(SPI2,(uint8_t*)user_data,strlen(user_data));

		//lets confirm SPI is not busy
		while( GetFlagStatus(SPI2,SPI_BSY_FLAG) );

		//Disable the SPI2 peripheral
		SPI_PeripheralControl(SPI2,DISABLE);
	}

	return 0;

}
