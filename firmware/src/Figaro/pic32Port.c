#include "pic32Port.h"

#define SYS_FREQ 200000000

volatile static uint32_t ucInterrupted = 0;       /* Flag to indicate interrupt occurred */

/**
	@brief Using SPI to transmit N bytes and return the received bytes. This function targets to 
         provide a more efficient way to transmit/receive data.
	@param pSendBuffer :{0 - 0xFFFFFFFF}
      - Pointer to the data to be sent.
	@param pRecvBuff :{0 - 0xFFFFFFFF}
      - Pointer to the buffer used to store received data.
	@param length :{0 - 0xFFFFFFFF}
      - Data length in SendBuffer.
	@return None.
**/
void AD5940_ReadWriteNBytes(unsigned char *pSendBuffer,unsigned char *pRecvBuff,unsigned long length)
{
  //SPI5_WriteRead (pSendBuffer, length, pRecvBuff, length);   
    uint32_t tx_count=0, rx_count=0;
    //pADI_SPI0->CNT = length;
    while(1)
    {
        uint32_t fifo_sta = SPI5STATbits.RXBUFELM;
               // pADI_SPI0->FIFO_STAT;
        if(rx_count < length)
        {
          //if(fifo_sta&0xf00){//there is data in RX FIFO.
            if(fifo_sta > 0)
            {//there is data in RX FIFO.
                *pRecvBuff++ = SPI5BUF;
                rx_count ++;
            } 
        }
    if(tx_count < length){
      if((fifo_sta&0xf) < 8){// there is space in TX FIFO.
        SPI5BUF = *pSendBuffer++;
        tx_count ++;
      }
    }
    if(rx_count == length && tx_count==length)
      break;  //done
    }
    while((SPI5STATbits.SPIBUSY) == 1);//wait for transfer done.
}

void AD5940_CsClr(void)
{
   AD5940_CS_Clear();
   while(AD5940_CS_Get());
}

void AD5940_CsSet(void)
{
   AD5940_CS_Set();
   while(!AD5940_CS_Get());
}

void AD5940_RstSet(void)
{
   AD5940_RESET_Set();
   while(!AD5940_RESET_Get());
}

void AD5940_RstClr(void)
{
   AD5940_RESET_Clear();
   while(AD5940_RESET_Get());
}

void delay_us(unsigned int us)
{
    // Convert microseconds us into how many clock ticks it will take
    us *= SYS_FREQ / 1000000 / 2; // Core Timer updates every 2 ticks

    _CP0_SET_COUNT(0); // Set Core Timer count to 0

    while (us > _CP0_GET_COUNT()); // Wait until Core Timer count reaches the number we calculated earlier
}

void delay_ms(int ms)
{
    delay_us(ms * 1000);
}

void AD5940_Delay10us(uint32_t time)
{
//    // Convert microseconds us into how many clock ticks it will take
//    time *= SYS_FREQ / 1000000 / 2; // Core Timer updates every 2 ticks, x 10
//
//    _CP0_SET_COUNT(0); // Set Core Timer count to 0
//
//    while (time > _CP0_GET_COUNT()); // Wait until Core Timer count reaches the number we calculated earlier
    
    //time/=10;
    if(time == 0) time =1;
    delay_us(time * 10);
}

uint32_t AD5940_GetMCUIntFlag(void)
{
   return ucInterrupted;
}

uint32_t AD5940_ClrMCUIntFlag(void)
{
   ucInterrupted = 0;
   return 1;
}

void changeIntHandler(GPIO_PIN pin, uintptr_t context)
{
    if(pin == GPIO_PIN_RB2)//AD5940 Interrupt
    {
        if(!AD5940_INTRPT_Get())//Falling Edge
            ucInterrupted = 1;
    }
}

/* Functions that used to initialize MCU platform */

uint32_t AD5940_MCUResourceInit(void *pCfg)
{
    GPIO_PinInterruptCallbackRegister(GPIO_PIN_RB2, &changeIntHandler, (uintptr_t)NULL);
    GPIO_PinInterruptEnable(GPIO_PIN_RB2);
    AD5940_CsSet();
    AD5940_RstSet();
  return 0;
}
