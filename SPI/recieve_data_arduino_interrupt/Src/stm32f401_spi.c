#include "../inc/stm32f401_spi.h"

static void		spi_txe_it_handle(SPI_Handler *pSPIHandle);
static void		spi_rxne_it_handle(SPI_Handler *pSPIHandle);
static void		spi_err_it_handle(SPI_Handler *pSPIHandle);

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




void SPI_Disable(SPI_REGDEF *pSPIx )
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

void SPI_IRQconf (uint8_t IRQnum , uint8_t ENORDS )
{
	 if (ENORDS == ENABLE)
	    {
					if (IRQnum <= 31 )
					{
						NVIC_ISER0 |= (1 << IRQnum );
					}
					else if (IRQnum > 31 && IRQnum < 64 )
					{

					  NVIC_ISER1 |= (1 << IRQnum % 32);
					}
					else if (IRQnum >= 64 && IRQnum < 96 )
					{

					  NVIC_ISER2 |= (1 << IRQnum % 64);
					}
	    }
	 else
	 	{
		 	 	 	if (IRQnum <= 31 )
		 			{
		 				NVIC_ISER0 &=~ (1 << IRQnum );
		 			}
		 			else if (IRQnum > 31 && IRQnum < 64 )
		 			{

		 			  NVIC_ISER1 &=~  (1 << IRQnum % 32);
		 			}
		 			else if (IRQnum >= 64 && IRQnum < 96 )
		 			{

		 			  NVIC_ISER2 &=~  (1 << IRQnum % 64);
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

void  SPI_PriorityConf ( uint8_t IRQpriority , uint8_t IRQnum)
{
   uint8_t iprx= IRQnum /4;
   uint8_t iprx_sec= IRQnum %4;
   uint8_t shiftAmount = (  8 * iprx_sec + 8 - NUM_PR_BITS_IMPLEMENTED);
   *( NVIC_IPR + (iprx)) |= (IRQpriority << shiftAmount ) ;
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

void SPI_IRQ_Handle (SPI_Handler *pSPIHandle)
{
	uint8_t temp1,temp2;
	temp1 = pSPIHandle -> pSPI -> SR  & ( 1 << SPI_SR_TXE) ;
	temp2 = pSPIHandle -> pSPI -> CR2 & ( 1 << SPI_CR2_TXEIE) ;

	if (temp1 && temp2 )
	{
		spi_txe_it_handle(pSPIHandle);

	}

	temp1 = pSPIHandle -> pSPI -> SR  & ( 1 << SPI_SR_RXNE) ;
	temp2 = pSPIHandle -> pSPI -> CR2 & ( 1 << SPI_CR2_RXNEIE) ;

	if (temp1 && temp2 )
	{
		spi_rxne_it_handle(pSPIHandle);
	}

	temp1 = pSPIHandle -> pSPI -> SR  & ( 1 << SPI_SR_OVR) ;
	temp2 = pSPIHandle -> pSPI -> CR2 & ( 1 << SPI_CR2_ERRIE) ;

	if (temp1 && temp2 )
	{
		spi_err_it_handle(pSPIHandle);
	}
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



uint8_t SPI_SendDataIT(SPI_Handler *pSPIHandle,uint8_t *pTxBuffer, uint32_t Len)
{
	uint8_t state = pSPIHandle -> TxState ;
	if (state != SPI_BSY_TX )
	{
 // 1 - SAVE THE ADRESS AND LEN

	pSPIHandle -> pTxBuffer = pTxBuffer ;
	pSPIHandle -> TxLen = Len ;

 // 2- MARK THE SPI STATE AS BUSY
	pSPIHandle -> TxState = SPI_BSY_TX;

//3-  ENABLE THE TEXEIE CONTROL
	pSPIHandle -> pSPI -> CR2 |= ( 1 << SPI_CR2_TXEIE);

	}
return state ;
}
uint8_t SPI_RecieveIT(SPI_Handler *pSPIHandle,uint8_t *pRxBuffer, uint32_t Len)
{
	uint8_t state = pSPIHandle -> RxState ;
		if (state != SPI_BSY_RX )
		{
	 // 1 - SAVE THE ADRESS AND LEN

		pSPIHandle -> pRxBuffer = pRxBuffer ;
		pSPIHandle -> RxLen = Len ;

	 // 2- MARK THE SPI STATE AS BUSY
		pSPIHandle -> RxState = SPI_BSY_RX;

	//3-  ENABLE THE TEXEIE CONTROL
		pSPIHandle -> pSPI -> CR2 |= ( 1 << SPI_CR2_RXNEIE);

		}
	return state ;
}

/* implement some helper fn */
static void	spi_txe_it_handle(SPI_Handler *pSPIHandle)
{
	if( (pSPIHandle ->pSPI->CR1 & ( 1 << SPI_CR1_DFF) ) )
			{
				//16 bit DFF
				//1. load the data in to the DR
				pSPIHandle ->pSPI->DR = *((uint16_t*)pSPIHandle -> pTxBuffer);
				pSPIHandle-> TxLen--;
				pSPIHandle-> TxLen--;
				(uint16_t*)pSPIHandle->pTxBuffer++;
			}
	else
			{
				//8 bit DFF
				pSPIHandle ->pSPI->DR = *(pSPIHandle -> pTxBuffer);
				pSPIHandle-> TxLen--;
				pSPIHandle->pTxBuffer++;
			}
	if ( ! pSPIHandle-> TxLen)
	{
		SPI_CloseTransmission(pSPIHandle);
		SPI_ApplicationEventCallback(pSPIHandle , SPI_EVENT_TX_CMPLT);


	}
}
static void	spi_rxne_it_handle(SPI_Handler *pSPIHandle)
{
if( (pSPIHandle ->pSPI->CR1 & ( 1 << SPI_CR1_DFF) ) )
			{
				//16 bit DFF
				//1. load the data in to the DR
				  *((uint16_t*)pSPIHandle -> pRxBuffer) = (uint16_t)pSPIHandle ->pSPI->DR;
				pSPIHandle-> RxLen--;
				pSPIHandle-> RxLen--;
				(uint16_t*)pSPIHandle->pRxBuffer++;
			}
	else
			{
				//8 bit DFF
		 *(pSPIHandle -> pRxBuffer) = (uint8_t)pSPIHandle ->pSPI->DR;
				pSPIHandle-> RxLen--;
				pSPIHandle->pRxBuffer++;
			}
	if ( ! pSPIHandle-> RxLen)
		{
		SPI_CloseReception(pSPIHandle);
		SPI_ApplicationEventCallback(pSPIHandle , SPI_EVENT_RX_CMPLT);
		}

}

static void	spi_err_it_handle(SPI_Handler *pSPIHandle)
{
	uint8_t temp;
 //clr the ovr flag
	if (pSPIHandle -> TxState != SPI_BSY_TX)
	{
		temp = pSPIHandle -> pSPI -> DR ;
		temp = pSPIHandle -> pSPI -> SR ;
		(void) temp ;
	}

	// infor the app
	SPI_ApplicationEventCallback(pSPIHandle , SPI_EVENT_OVR_ERR);
}



void SPI_CloseTransmission (SPI_Handler *pSPI_handler)
{
	pSPI_handler->pSPI -> CR2 &= ~ ( 1 << SPI_CR2_TXEIE) ;
	pSPI_handler->pTxBuffer =NULL ;
	pSPI_handler-> TxLen = 0;
	pSPI_handler-> TxState = SPI_READY;
}
void SPI_CloseReception(SPI_Handler *pSPI_handler)
{
	pSPI_handler->pSPI -> CR2 &= ~ ( 1 << SPI_CR2_RXNEIE) ;
	pSPI_handler->pRxBuffer =NULL ;
	pSPI_handler-> RxLen = 0;
	pSPI_handler-> RxState = SPI_READY;
}


void SPI_ClearOVRFlag(SPI_Handler *pSPI_handler)
{
	uint8_t temp;
	temp = pSPI_handler -> pSPI -> DR ;
	temp = pSPI_handler -> pSPI -> SR ;
	(void) temp ;
}

__WEAK void SPI_ApplicationEventCallback(SPI_Handler *pSPI_handler , uint8_t AppEv)
{

}
