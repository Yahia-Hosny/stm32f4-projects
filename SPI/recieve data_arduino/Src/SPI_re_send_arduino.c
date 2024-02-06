/*
 * SPI_re_send_arduino.c
 *
 *  Created on: Feb 6, 2024
 *      Author: MozaKRA
 */
#include<string.h>
#include "stm32f401_spi.h"

#define COMMAND_LED_CRTL          0x50
#define COMMAND_SENSOR_READ       0x51
#define COMMAND_LED_READ          0x52
#define COMMAND_PRINT             0x53
#define COMMAND_ID_READ           0x54

#define LED_ON           1
#define LED_OFF          0

// ARDUINO ANALOG PINS
#define ANALOG_PIN0          0
#define ANALOG_PIN1          1
#define ANALOG_PIN2          2
#define ANALOG_PIN3          3
#define ANALOG_PIN4          4


#define LED_PIN          9

void delay()
{
	for(uint32_t i = 0 ; i < 500000/2 ; i ++);
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
void spi_led_ctrl()
{
	uint8_t dummy_write = 0xff;
	uint8_t dummy_read ;
	uint8_t ackbyte ;
	uint8_t args[2] = { LED_PIN , LED_ON};
	uint8_t commandCode = COMMAND_LED_CRTL;
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
		args[0] = LED_PIN  ;
		// led on or off
		args[1] = LED_ON ;
		delay();
		SPI_SendData(SPI2,args,2);
		SPI_Recieve(SPI2,&dummy_read,2);
	}

}
void spi_sens_read ()
{
    uint8_t dummy_write = 0xff;
	uint8_t dummy_read ;
	uint8_t ackbyte ;
	uint8_t args;
	uint8_t commandCode = COMMAND_SENSOR_READ;
	uint8_t sensor_value ;
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
		args = ANALOG_PIN0  ;
		SPI_SendData(SPI2,&args,1);
		SPI_Recieve(SPI2,&dummy_read,1);
		delay();
		SPI_SendData(SPI2,&dummy_write,1);
		SPI_Recieve(SPI2,&sensor_value,1);
		delay();
		SPI_SendData(SPI2,&sensor_value,1);
		SPI_Recieve(SPI2,&dummy_read,1);

	}
}
void spi_led_read ()
{
    uint8_t dummy_write = 0xff;
	uint8_t dummy_read ;
	uint8_t ackbyte ;
	uint8_t args;
	uint8_t commandCode = COMMAND_LED_READ;
	uint8_t Led_value ;
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
		args = LED_PIN ;
		SPI_SendData(SPI2,&args,1);
		SPI_Recieve(SPI2,&dummy_read,1);
		delay();
		SPI_SendData(SPI2,&dummy_write,1);
		SPI_Recieve(SPI2,&Led_value,1);
		delay();
		SPI_SendData(SPI2,&Led_value,1);
		SPI_Recieve(SPI2,&dummy_read,1);

	}
}
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
			SPI_PeripheralControl(SPI2,ENABLE);

			// turn on the led in pin 9
            spi_led_ctrl();

            while( ! GPIO_ReadPin(GPIOA,GPIO_PIN_0) );
           //to avoid button de-bouncing related issues 200ms of delay
            delay();
            // read the sensor value  on A0
            spi_sens_read ();


            while( ! GPIO_ReadPin(GPIOA,GPIO_PIN_0) );
            //to avoid button de-bouncing related issues 200ms of delay
             delay();
             // read the led value  on led_pin
             spi_led_read ();

			// wait until the spi finish sending data
			while( GetFlagStatus(SPI2,SPI_BSY_FLAG) );
			//Disable the SPI2 peripheral
			SPI_PeripheralControl(SPI2,DISABLE);
		}

		return 0 ;

}
