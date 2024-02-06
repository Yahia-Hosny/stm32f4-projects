#include "../inc/stm32f401_spi.h"



/********************get flag status *************************/
uint8_t GetFlagStatus(SPI_REGDEF *pSPI , uint32_t FlagName)
{
	if(pSPI->SR & FlagName)
		{
			return FLAG_SET;
		}
		return FLAG_RESET;
}


/*********************************************************************
* @fn      		     -
*
* @brief             -
*
* @param[in]         -
* @param[in]         -
* @param[in]         -
*
* @return            -  none
*
* @Note              -  none

 */

void SPI_clkCntrl(SPI_REGDEF *pSpi  , uint8_t EnOrDsi)
{
   if (EnOrDsi == ENABLE)
   {
     if(pSpi == SPI1 )
     {
    	 SPI1_PCLK_EN();
     }
     else if(pSpi == SPI2 )
     {
    	 SPI2_PCLK_EN();
     }
     else if(pSpi == SPI3 )
     {
    	 SPI3_PCLK_EN();
     }
     else if(pSpi == SPI4 )
     {
    	 SPI4_PCLK_EN();
     }

   }
   else
   {
     if(pSpi == SPI1 )
     {
    	 SPI1_PCLK_DS();
     }
     else if(pSpi == SPI2 )
     {
    	 SPI2_PCLK_DS();
     }
     else if(pSpi == SPI3 )
     {
    	 SPI3_PCLK_DS();
     }
     else if(pSpi == SPI4 )
     {
    	 SPI4_PCLK_DS();
     }

   }
}


/*********************************************************************
* @fn      		     -
*
* @brief             -
*
* @param[in]         -
* @param[in]         -
* @param[in]         -
*
* @return            -  none
*
* @Note              -  none

 */

void SPI_init(SPI_Handler *pSPI_handler)
{

	//peripheral clock enable
	SPI_clkCntrl(pSPI_handler -> pSPI , ENABLE);

  // lets configure the SPI_CR1 register
   uint32_t temp = 0;
   //1. configure the device mode
   	temp|= (pSPI_handler -> SpiConfig.SPI_DeviceMode << SPI_CR1_MSTR) ;

   	//2. Configure the bus config
   if (pSPI_handler -> SpiConfig .SPI_BusConfg == SPI_BUS_CONFIG_FD)
   {
     // enable unidirectional line
     temp &= ~(1 << SPI_CR1_BIDI_MODE);

   }
  else if (pSPI_handler -> SpiConfig .SPI_BusConfg == SPI_BUS_CONFIG_HD)
   {
     // enable bidirectional line
     temp  |= (1 << SPI_CR1_BIDI_MODE);

   }
    else if (pSPI_handler -> SpiConfig .SPI_BusConfg == SPI_BUS_CONFIG_SIMPLEX_RX)
   {
      // enabble unidirectional line
     temp &= ~(1 << SPI_CR1_BIDI_MODE);
     // enable recieve only mode
     temp |= (1<< SPI_CR1_RX_ONLY) ;

   }

   // configure the clock speed

    temp |= (pSPI_handler -> SpiConfig.SPI_Speed << SPI_CR1_BR);

   // configure the DFF
   temp |= (pSPI_handler -> SpiConfig.SPI_DFF << SPI_CR1_DFF);

    // configure the CPHA
   temp |= (pSPI_handler -> SpiConfig.SPI_CPHA << SPI_CR1_CPHA);

   // configure the CPOL
   temp |= (pSPI_handler -> SpiConfig.SPI_CPOL << SPI_CR1_CPOL);

   // configure the SSM

  temp |=  pSPI_handler -> SpiConfig.SPI_SSM << SPI_CR1_SSM;




   // finish the configure
   pSPI_handler -> pSPI -> CR1 = temp ;



}


/*********************************************************************
* @fn      		       - SPI_Deinit
*
* @brief             -RESET THE REGISTER OF THE SPIx
*
* @param[pSPI]         - POINTER TO REGDEF STRUCT  @SPI_MACROS
*

*
* @return            -  none
*
* @Note              -  none

 */

void SPI_Deinit(SPI_REGDEF *pSPI)
{
  if (pSPI == SPI1)
    {
        SPI1_RESET;
    }
    else if (pSPI == SPI2)
    {
        SPI2_RESET;
    }
    else if (pSPI == SPI3)
    {
        SPI3_RESET;
    }
    else if (pSPI == SPI4)
    {
        SPI4_RESET;
    }
}




void SPI_Disable(SPI_REGDEF *pSPIx)
{
	pSPIx -> CR1 &= ~ (1 << SPI_CR1_SPE );
}

/*********************************************************************
* @fn      		       - SPI_Send
*
* @brief             - SEND DATA THROUGH THE SPI
*
* @param[pSPI]         - POINTER TO REGDEF STRUCT  @SPI_MACROS
* @param[pTxBuffer]         -  POINTER TO  THE DATA
* @param[len]         - LEN OF DATA
*
* @return            -  none
*
* @Note              -  none

 */



void SPI_SendData(SPI_REGDEF *pSPIx,uint8_t *pTxBuffer, uint32_t Len)
{
	if ( (pSPIx->CR1 & SPI_CR1_SPE) != SPI_CR1_SPE)
	  {
	    /* Enable SPI peripheral */
		pSPIx->CR1 |= (1 << SPI_CR1_SPE);
	  }

	while(Len > 0)
	{
		//1. wait until TXE is set
		while( (GetFlagStatus(pSPIx,SPI_TXE_FLAG))  == FLAG_RESET );

		//2. check the DFF bit in CR1
		if( (pSPIx->CR1 & ( 1 << SPI_CR1_DFF) ) )
		{
			//16 bit DFF
			//1. load the data in to the DR
			pSPIx->DR = *((uint16_t*)pTxBuffer);
			Len--;
			Len--;
			(uint16_t*)pTxBuffer++;
		}else
		{
			//8 bit DFF
			pSPIx->DR = *pTxBuffer;
			Len--;
			pTxBuffer++;
		}
	}

}

/*********************************************************************
* @fn      		     -
*
* @brief             -
*
* @param[in]         -
* @param[in]         -
* @param[in]         -
*
* @return            -  none
*
* @Note              -  none

 */

void SPI_Recieve(SPI_REGDEF *pSPIx,uint8_t *pRxBuffer, uint32_t Len)
{
  while(Len > 0)
		{
	  //1. wait until RXNE is set
	  			while(GetFlagStatus(pSPIx,SPI_RXNE_FLAG)  == (uint8_t)FLAG_RESET );

	  			//2. check the DFF bit in CR1
	  			if( (pSPIx->CR1 & ( 1 << SPI_CR1_DFF) ) )
	  			{
	  				//16 bit DFF
	  				//1. load the data from DR to Rxbuffer address
	  				 *((uint16_t*)pRxBuffer) = pSPIx->DR ;
	  				Len--;
	  				Len--;
	  				(uint16_t*)pRxBuffer++;
	  			}
	  			else
	  			{
	  				//8 bit DFF
	  				*(pRxBuffer) = pSPIx->DR ;
	  				Len--;
	  				pRxBuffer++;
	  			}
	  		}

}


/*********************************************************************
* @fn      		     -
*
* @brief             -
*
* @param[in]         -
* @param[in]         -
* @param[in]         -
*
* @return            -  none
*
* @Note              -  none

 */

uint8_t SPI_IRQconf (uint8_t IRQnum , uint8_t ENORDS )
{

}


/*********************************************************************
* @fn      		     -
*
* @brief             -
*
* @param[in]         -
* @param[in]         -
* @param[in]         -
*
* @return            -  none
*
* @Note              -  none

 */

uint8_t SPI_PriorityConf ( uint8_t IRQpriority , uint8_t IRQnum)
{

}


/*********************************************************************
* @fn      		     -
*
* @brief             -
*
* @param[in]         -
* @param[in]         -
* @param[in]         -
*
* @return            -  none
*
* @Note              -  none

 */

void SPI_IRQ_Handle (uint8_t pinNum)
{

}

void SPI_PeripheralControl(SPI_REGDEF *pSPIx , uint8_t EnOrDsi)
{
	if ( EnOrDsi == ENABLE )
	{
		pSPIx -> CR1 |= (1 << SPI_CR1_SPE ) ;
	}


	else
	{
		pSPIx -> CR1  &= ~ (1 << SPI_CR1_SPE ) ;
	}
}


/*******************************************************************************/

void SPI_ssi_conf(SPI_REGDEF *pSPIx , uint8_t EnOrDsi)
{
	if ( EnOrDsi == ENABLE )
		{
			pSPIx -> CR1 |= (1 << SPI_CR1_SSI ) ;
		}


		else
		{
			pSPIx -> CR1  &= ~ (1 << SPI_CR1_SSI ) ;
		}
}



void SPI_ssoe_conf(SPI_REGDEF *pSPIx , uint8_t EnOrDsi)
{
	if ( EnOrDsi == ENABLE )
		{
			pSPIx -> CR2 |= (1 << SPI_CR2_SSOE ) ;
		}


		else
		{
			pSPIx -> CR2  &= ~ (1 << SPI_CR2_SSOE  ) ;
		}
}
