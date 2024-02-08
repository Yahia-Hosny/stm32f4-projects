/*
 * spi_recievedata_it.c
 *
 *  Created on: Feb 8, 2024
 *      Author: MozaKRA
 */
#include<string.h>
#include "stm32f401_spi.h"
#define COMMAND_PRINT             0x53
void delay()
{
	for(uint32_t i = 0 ; i < 500000/2 ; i ++); // 100 ms
}
uint8_t SPI_VerifyResponse(uint8_t ackbyte)
{

	if(ackbyte == (uint8_t)0xFA)
	{
		//ack
		return 1;
	}

	return 0;
}
SPI_Handler SPI2handle;
#define MAX_LEN 500
char RcvBuff[MAX_LEN];
volatile char ReadByte;
volatile uint8_t rcvStop = 0;
/*This flag will be set in the interrupt handler of the Arduino interrupt GPIO */
volatile uint8_t dataAvailable = 0;
void SPI2_GPIOInits(void)
{
	GPIO_Handler SPIPins;

	SPIPins.GPIOX = GPIOB;
	SPIPins.GPIO_Pin_Conf.PinMode  = GPIO_MODE_ALTFN;
	SPIPins.GPIO_Pin_Conf.PinAltFn = 5;
	SPIPins.GPIO_Pin_Conf.PinOPTYP = GPIO_OUTYPE_PP;
	SPIPins.GPIO_Pin_Conf.PinPUPDR = GPIO_PUPD_NO;
	SPIPins.GPIO_Pin_Conf.PinSpeed = GPIO_PINSPEED_HIGH;

	//SCLK
	SPIPins.GPIO_Pin_Conf.PinNum = GPIO_PIN_13;
	GPIO_init(&SPIPins);

	//MOSI
	SPIPins.GPIO_Pin_Conf.PinNum  = GPIO_PIN_15;
	GPIO_init(&SPIPins);


	//MISO
	SPIPins.GPIO_Pin_Conf.PinNum  = GPIO_PIN_14;
		GPIO_init(&SPIPins);


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
void Slave_GPIO_InterruptPinInit(void)
{
	     GPIO_Handler spiIntPin;
		memset(&spiIntPin,0,sizeof(spiIntPin));

		//this is led gpio configuration
		spiIntPin.GPIOX = GPIOD;
		spiIntPin.GPIO_Pin_Conf.PinNum  = GPIO_PIN_2 ;
		spiIntPin.GPIO_Pin_Conf.PinMode = GPIO_MODE_IT_FT;
		spiIntPin.GPIO_Pin_Conf.PinSpeed= GPIO_PINSPEED_LOW;
		spiIntPin.GPIO_Pin_Conf.PinPUPDR= GPIO_PUPD_PU;

		GPIO_init(&spiIntPin);

		GPIO_PriorityConf(IRQ_NUM_EXTI2, IRQ_PRIORITY_15);
		GPIO_IRQconf(IRQ_NUM_EXTI2, ENABLE);

}
void spi_print (uint8_t buffer[] , uint8_t len )
{
    uint8_t dummy_write = 0xff;
	uint8_t dummy_read ;
	uint8_t ackbyte ;
	uint8_t args;
	uint8_t commandCode = COMMAND_PRINT;
	// send the command to the slave
	SPI_SendData(SPI2, &commandCode, 1);
	// do dummy read to clear off the RXNE
	SPI_Recieve(SPI2, &dummy_read, 1);
	// send dummy byte to recieve response
	SPI_SendData(SPI2, &dummy_write, 1);
	// read ackbyte
	SPI_Recieve(SPI2, &ackbyte, 1);
	//check ackbyte
	if(SPI_VerifyResponse(ackbyte))
	{
	 //send arguments
	//pin number
		args = len ;
		SPI_SendData(SPI2,&args,1);
		SPI_Recieve(SPI2,&dummy_read,1);
		delay();
		SPI_SendData(SPI2,&dummy_write,1);

		 for(int i=0 ; i < args ; i++)
		    {
				SPI_SendData(SPI2,&buffer[i],1);
				SPI_Recieve(SPI2,&dummy_read,1);
		    }

	}
}

int main(void)
{
	uint8_t dummy = 0xff;

	Slave_GPIO_InterruptPinInit();

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

	SPI_IRQconf(IRQ_NUM_SPI2, ENABLE);

while(1)

{
	rcvStop = 0;

	while(!dataAvailable); //wait till data available interrupt from transmitter device(slave)

	GPIO_IRQconf(IRQ_NUM_EXTI2,DISABLE);

	//enable the SPI2 peripheral
	SPI_PeripheralControl(SPI2,ENABLE);


	while(!rcvStop)
	{
		/* fetch the data from the SPI peripheral byte by byte in interrupt mode */
		while ( SPI_SendDataIT(&SPI2handle,&dummy,1) == SPI_BSY_TX);
		while ( SPI_RecieveIT(&SPI2handle,&ReadByte,1) == SPI_BSY_RX );
	}

	// confirm SPI is not busy
	while( GetFlagStatus(SPI2,SPI_BSY_FLAG) );

	//Disable the SPI2 peripheral
	SPI_PeripheralControl(SPI2,DISABLE);

	dataAvailable = 0;

	GPIO_IRQconf(IRQ_NUM_EXTI2,ENABLE);

}

return 0 ;

}

void SPI2_IRQHandler(void)
{

	SPI_IRQ_Handle(&SPI2handle);
}
void SPI_ApplicationEventCallback(SPI_Handler *pSPIHandle,uint8_t AppEv)
{
	static uint32_t i = 0;
	/* In the RX complete event , copy data in to rcv buffer . '\0' indicates end of message(rcvStop = 1) */
	if(AppEv == SPI_EVENT_RX_CMPLT)
	{
				RcvBuff[i++] = ReadByte;
				if(ReadByte == '\0' || ( i == MAX_LEN)){
					rcvStop = 1;
					RcvBuff[i-1] = '\0';
					i = 0;
				}
	}

}

void EXTI2_IRQHandler(void)
{
	GPIO_IRQ_Handle(GPIO_PIN_2);
	dataAvailable = 1;
}
