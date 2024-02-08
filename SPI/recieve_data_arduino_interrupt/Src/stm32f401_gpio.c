
#include "stm32f401_gpio.h"

/*********************************************************************
* @fn      		     - GPIO_clkCntrl
*
* @brief             - enable or disable the clock for gpio port
*
* @param[pGpio]      -  pointer to  gpio struct
* @param[EnOrDsi]    - micro enable or disable the clock
*
*
* @return            -  none
*
* @Note              -  none

 */
void GPIO_clkCntrl(GPIO_REGDEF *pGpio, uint8_t EnOrDsi)
{
    if (EnOrDsi == ENABLE)
    {
        if (pGpio == GPIOA)
        {
            GPIOA_PCLK_EN;
        }
        else if (pGpio == GPIOB)
        {
            GPIOB_PCLK_EN;
        }
        else if (pGpio == GPIOC)
        {
            GPIOC_PCLK_EN;
        }
        else if (pGpio == GPIOD)
        {
            GPIOD_PCLK_EN;
        }
    }

    else
    {
        if (pGpio == GPIOA)
        {
            GPIOA_PCLK_DS;
        }
        else if (pGpio == GPIOB)
        {
            GPIOB_PCLK_DS;
        }
        else if (pGpio == GPIOC)
        {
            GPIOC_PCLK_DS;
        }
        else if (pGpio == GPIOD)
        {
            GPIOD_PCLK_DS;
        }
    }
}

/*********************************************************************
* @fn      		     - GPIO_init
*
* @brief             - initialize the gpio port
*
* @param[pGpioX]      -  pointer to  gpio handler  struct
*
*
* @return            -  none
*
* @Note              -  none

 */
void GPIO_init(GPIO_Handler *pGpioHandler)

{ // pin number
	 GPIO_clkCntrl(pGpioHandler ->GPIOX, ENABLE);
	 uint32_t temp =0 ;
    if (pGpioHandler->GPIO_Pin_Conf.PinMode <= GPIO_MODE_ANALOG)
    {
        // pin mode

        temp |= ((pGpioHandler->GPIO_Pin_Conf.PinMode) << (2 * (pGpioHandler->GPIO_Pin_Conf.PinNum)));
        pGpioHandler->GPIOX->MODER &= ~((0x3) << (pGpioHandler->GPIO_Pin_Conf.PinNum));
        pGpioHandler->GPIOX->MODER |= temp;

        // pin speed
        temp |= ((pGpioHandler->GPIO_Pin_Conf.PinSpeed) << (2 * (pGpioHandler->GPIO_Pin_Conf.PinNum)));
        pGpioHandler->GPIOX->OSPEEDR &= ~((0x3) << (pGpioHandler->GPIO_Pin_Conf.PinNum));
        pGpioHandler->GPIOX->OSPEEDR |= temp;

        // pin pupd control

        temp |= ((pGpioHandler->GPIO_Pin_Conf.PinPUPDR) << (2 * (pGpioHandler->GPIO_Pin_Conf.PinNum)));
        pGpioHandler->GPIOX->PUPDR &= ~((0x3) << (2 * pGpioHandler->GPIO_Pin_Conf.PinNum));
        pGpioHandler->GPIOX->PUPDR |= temp;

        // pin otype control

        temp |= ((pGpioHandler->GPIO_Pin_Conf.PinOPTYP) << ((pGpioHandler->GPIO_Pin_Conf.PinNum)));
        pGpioHandler->GPIOX->OTYPER &= ~((0x3) << (pGpioHandler->GPIO_Pin_Conf.PinNum));
        pGpioHandler->GPIOX->OTYPER |= temp;

        // pin alternate fn  control
        if ((pGpioHandler->GPIO_Pin_Conf.PinMode) == GPIO_MODE_ALTFN)
        {
            // config for the alt fn
            uint8_t temp1, temp2;
            temp1 = (pGpioHandler->GPIO_Pin_Conf.PinNum) / 8;
            temp2 = (pGpioHandler->GPIO_Pin_Conf.PinNum) % 8;
            pGpioHandler->GPIOX->AFR[temp1] &= ~((0xF) << (temp2));
            pGpioHandler->GPIOX->AFR[temp1] |= ((pGpioHandler->GPIO_Pin_Conf.PinAltFn) << (4 * (temp2)));
        }
    }
    else
        {
            if (pGpioHandler->GPIO_Pin_Conf.PinMode == GPIO_MODE_IT_FT )
            {
                // CONF THE FALLING EDGE R
                EXTI ->FTSR |= (1 <<pGpioHandler->GPIO_Pin_Conf.PinNum ) ;
                // CLEAR THE RISING EDGE R
                EXTI ->RTSR &= ~(1 <<pGpioHandler->GPIO_Pin_Conf.PinNum ) ;
            }
            else if (pGpioHandler->GPIO_Pin_Conf.PinMode == GPIO_MODE_IT_RT)
            {
                // CONF THE RISING EDGE R
                EXTI ->RTSR |= (1 <<pGpioHandler->GPIO_Pin_Conf.PinNum ) ;
                // CLEAR THE FALLING  EDGE R
                EXTI ->FTSR &= ~(1 <<pGpioHandler->GPIO_Pin_Conf.PinNum ) ;
            }
            else if (pGpioHandler->GPIO_Pin_Conf.PinMode == GPIO_MODE_IT_RFT)
            {
                // CONF THE RISING & FALLING  EDGE R
                EXTI ->RTSR |=  (1 <<pGpioHandler->GPIO_Pin_Conf.PinNum ) ;
                EXTI ->FTSR |=  (1 <<pGpioHandler->GPIO_Pin_Conf.PinNum ) ;
            }

                // PORT SELECTION
                uint8_t temp4  =pGpioHandler->GPIO_Pin_Conf.PinNum / 4;
                uint8_t temp5 =pGpioHandler->GPIO_Pin_Conf.PinNum % 4;
                uint8_t pinCode = GPIO_BASE_CODE(pGpioHandler->GPIOX) ;
                SYSCFG_PCLK_EN;
                SYSCFG -> EXTICR[temp4] = (pinCode << temp5 * 4 );
                // ENABLE INTERRUPT DELIVERY
                EXTI ->IMR |= ( 1 <<pGpioHandler->GPIO_Pin_Conf.PinNum );


        }
}

/*********************************************************************
* @fn      		     - GPIO_Deinit
*
* @brief             - RESET THE REGISTER OF THE GPIO PORT
*
* @param[in]         - POINTER TO REGDEF STRUCT  @GPIO_MACROS
*
* @return            -  none
*
* @Note              -  none

 */

void GPIO_Deinit(GPIO_REGDEF *pGpio)
{
    if (pGpio == GPIOA)
    {
        GPIOA_RESET;
    }
    else if (pGpio == GPIOB)
    {
        GPIOB_RESET;
    }
    else if (pGpio == GPIOC)
    {
        GPIOC_RESET;
    }
    else if (pGpio == GPIOD)
    {
        GPIOD_RESET;
    }
}

/*********************************************************************
* @fn      		     - GPIO_ReadPin
*
* @brief             - read from input pin
*
* @param[pGpiox]         - pointer to  gpio regDef
* @param[pinNum]         - pin number
*
* @return            -  uint8_t value
*
* @Note              -  none

 */
uint8_t GPIO_ReadPin(GPIO_REGDEF *pGpiox, uint8_t pinNum)
{
    uint8_t value;
    value = (uint8_t)(((pGpiox->IDR) >> pinNum) & (BIT_MASK_8));
    return value;
}

/*********************************************************************
* @fn      		     - GPIO_ReadPort
*
* @brief             - read the entire gpio port
*
* @param[pGpiox]         - pointer to  gpio regDef
*
* @return            -  uint16_t value
*
* @Note              -  none

 */
uint16_t GPIO_ReadPort(GPIO_REGDEF *pGpiox)
{
    uint16_t value = (uint16_t)(pGpiox->IDR);
    return value;
}

/*********************************************************************
* @fn      		     - writepin
*
* @brief             - write to specific gpio pin
*
* @param[pGpiox]         - pointer to  gpio regDef
* @param[pinNum]         - pin number
* @param[value]         -  value want to write to the pin
*
* @return            -  none
*
* @Note              -  none

 */

void GPIO_WritePin(GPIO_REGDEF *pGpiox, uint8_t pinNum, uint8_t value)
{
    if (value == GPIO_PIN_SET)
    {
        pGpiox->ODR |= (1 << pinNum);
    }
    else
    {
        pGpiox->ODR &= ~(1 << pinNum);
    }
}

/*********************************************************************
* @fn      		    - writeport
*
* @brief             - write to specific gpio port
*
* @param[pGpiox]         - pointer to  gpio regDef
* @param[value]          -  uint_16 value want to write to the port
*
* @return                -   none
*
* @Note              -  none

 */

void GPIO_WritePort(GPIO_REGDEF *pGpiox, uint16_t value)
{
   pGpiox -> ODR = value ;
}
/*********************************************************************
* @fn      		    - toggpin
*
* @brief             - toggle output gpio pin
*
* @param[pGpiox]         - pointer to  gpio regDef
* @param[pinNum]         - pin number
*
* @return                -   none
*
* @Note              -  none

 */
void GPIO_ToggPin(GPIO_REGDEF *pGpiox  , uint8_t pinNum  )
{

    pGpiox -> ODR  ^= (1<<pinNum);
}

/*********************************************************************
* @fn      		    - toggport
*
* @brief             - toggle output gpio port
*
* @param[pGpiox]         - pointer to  gpio regDef
*
* @return                -   none
*
* @Note              -  none

 */
void  GPIO_ToggPort(GPIO_REGDEF *pGpiox )
{
    for (uint8_t i = 0; i < GPIO_PINNumbers ; i++)
    {
        pGpiox -> ODR ^= ( 1 << i) ;
    }

}
/*********************************************************************
* @fn      		     - GPIO_IRQconf
*
* @brief             - configure the IRQ
*
* @param[IRQnum]             - macros of irq number
* @param[IRQpriority]         - irq pariority
* @param[ENORDS]               -  macros of enable or disable
*
* @return            -  uint8_t
*
* @Note              -  none

 */

void GPIO_IRQconf (uint8_t IRQnum ,uint8_t ENORDS )
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
                NVIC_ICER0 |= (1 << IRQnum );
            }
            else if (IRQnum > 31 && IRQnum < 64 )
            {

              NVIC_ICER1 |= (1 << IRQnum % 32);
            }
            else if (IRQnum >= 64 && IRQnum < 96 )
            {

              NVIC_ICER2 |= (1 << IRQnum % 64);
            }

    }
}

/*********************************************************************
* @fn      		     - GPIO_priorityconf
*
* @brief             - configure the priority
*
* @param[IRQnum]             - macros of irq number
* @param[IRQpriority]         - irq pariority
*
* @return            -  uint8_t
*
* @Note              -  none

 */
void GPIO_PriorityConf ( uint8_t IRQpriority , uint8_t IRQnum)
{
   uint8_t iprx= IRQnum /4;
   uint8_t iprx_sec= IRQnum %4;
   uint8_t shiftAmount = (  8 * iprx_sec + 8 - NUM_PR_BITS_IMPLEMENTED);
   *( NVIC_IPR + (iprx)) |= (IRQpriority << shiftAmount ) ;
}

/*********************************************************************
* @fn      		     - GPIO_IRQ_Handle
*
* @brief             - handle  the IRQ
*
* @param[pinNum]     - GPIO pin number
*
* @return            -  none
*
* @Note              -  none

 */

void GPIO_IRQ_Handle (uint8_t pinNum)
{
    // clear exti pr reg
    if ( EXTI ->PR & (1 << pinNum))
    {
        // clear the bit
        EXTI ->PR |= (1 << pinNum);
    }
}


