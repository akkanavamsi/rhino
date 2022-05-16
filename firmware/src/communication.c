#include "communication.h"
#include "nmea.h"
#include "modbus.h"
#include "anemometer.h"

#define UNLOCK_KEY_1 0xAA996655 
#define UNLOCK_KEY_2 0x556699AA
#define UNLOCK_KEY_CLEAR 0x00000000

void uart6_comTasks(void);
void uart6_counters(void);
void uart2_comTasks(void);
void uart2_counters(void);

//*********** All UARTS *********** 
void communication_init(void)
{
    uartComData6.state = UART_BUFFER_INIT;
    uartComData2.state = UART_BUFFER_INIT;
}

void communication_task(void)
{
    uart6_comTasks();
    uart2_comTasks();
}

void uart_counters(void)
{
    uart6_counters();
    uart2_counters();
}

//*********** UART 6 mDot *********** 
void clearSendBufferUart6(void);
void clearReceiveBufferUart6(void);
void sendBufferUart6(void);
bool uart6Busy(void);
void uart6_comHandlingDone(void);

UART_COM_DATA uartComData6;
uint8_t rxByteU6;
uint16_t uart6_timeout = 0;
uint16_t uart6_eof = 0;
bool uart6_timeoutFlag = false;
bool uart6_eofFlag = false;

uint16_t tempU2Counter = 0;

void uart6_writeCallback(uintptr_t context)
{ 
    uartComData6.state = UART_BUFFER_WAIT;
    uartComData6.writeStatus = true;
}

void uart6_readCallback(uintptr_t context)
{         
    UART6_Read(&rxByteU6, 1);
    uartComData6.receiveBuffer[uartComData6.receiveCount++] = rxByteU6;
    uart6_eof = 0;
    uartComData6.state = UART_RECEIVE_BUFFER;
}

void uart6_counters(void)
{
    if((uartComData6.state == UART_BUFFER_WAIT) || (uartComData6.state == UART_RECEIVE_BUFFER))
    {
        uart6_timeout++;
        if(uart6_timeout >= 1000)//1000ms
        {
            uart6_timeout = 0;
            uart6_timeoutFlag = true;
        }
    }
    if(uartComData6.state == UART_RECEIVE_BUFFER)//check eof
    {
        uart6_eof++;
        if(uart6_eof >= 10)//100ms
        {
            uart6_eofFlag = true;
            uart6_eof = 0;
        }
    }
}

void uart6_comHandlingDone(void)
{
    uartComData6.readStatus = false;
    uartComData6.state = UART_BUFFER_IDLE;
}

void uart6_comTasks(void)
{
    switch(uartComData6.state)
    {
        case UART_BUFFER_INIT:
            UART6_WriteCallbackRegister(uart6_writeCallback, 0);
            UART6_ReadCallbackRegister(uart6_readCallback, 0);
            uartComData6.writeStatus = false;
            uartComData6.readStatus = false;
            uartComData6.timeutStatus = false;
            clearSendBufferUart6();
            clearReceiveBufferUart6();
            UART6_Read(&rxByteU6, 1); 
            uart6_eof = 0;
            uart6_timeout = 0;
            uart6_timeoutFlag = false;
            uart6_eofFlag = false;
            uartComData6.state = UART_BUFFER_IDLE;
            break;
        case UART_SEND_BUFFER:
            clearReceiveBufferUart6();
            UART6_Write(uartComData6.sendBuffer, uartComData6.sendCount);
            uart6_timeout = 0;
            uart6_eof = 0;
            uart6_timeoutFlag = false;
            uart6_eofFlag = false;
            uartComData6.timeutStatus = false;
            uartComData6.readStatus = false;
            uartComData6.state = UART_BUFFER_WAIT;
            break;
        case UART_RECEIVE_BUFFER:
            //if(uartComData6.receiveBuffer[uartComData6.receiveCount - 1] == 0x0A)//Line Feed
            //if(uartComData6.receiveBuffer[uartComData6.receiveCount -1] == 0x00)//NULL
            if(uart6_eofFlag)
            {
                uartComData6.state = UART_RECEIVE_PROCESS;
                uartComData6.readStatus = true;//EOF
                uart6_eofFlag = false;
            }
            if(uart6_timeoutFlag)
            {
                uartComData6.state = UART_BUFFER_ERROR;
            }
            break;
        case UART_RECEIVE_PROCESS:
//            clearReceiveBuffer();
            uartComData6.state = UART_BUFFER_IDLE;
            break;
        case UART_BUFFER_ERROR:
            uartComData6.timeout++;
            uartComData6.readStatus = true;
            uartComData6.timeutStatus = true;
            clearReceiveBufferUart6();
            uartComData6.state = UART_BUFFER_IDLE;
            break;
        case UART_BUFFER_WAIT:
            if(uart6_timeoutFlag)
            {
                uartComData6.state = UART_BUFFER_ERROR;
            }
//            if(!UART6_WriteIsBusy())              
//                uartComData6.state = UART_BUFFER_IDLE;
            break;
        default:
        case UART_BUFFER_IDLE:
            break;
    }
}

void clearSendBufferUart6(void)
{
    memset(uartComData6.sendBuffer,0,COM_BUFFER_SIZE);
    uartComData6.sendCount = 0;
}

void clearReceiveBufferUart6(void)
{
    memset(uartComData6.receiveBuffer,0,COM_BUFFER_SIZE);
    uartComData6.receiveCount = 0;
}

void sendBufferUart6(void)
{
    if(uartComData6.state == UART_BUFFER_IDLE)
        uartComData6.state = UART_SEND_BUFFER;
}

bool uart6Busy(void)
{
    if(uartComData6.state == UART_BUFFER_IDLE)
        return false;
    else
        return true;
}

uint16_t sendStringU6(char * str)
{
    uint16_t len = strlen(str);
    int i;
    clearSendBufferUart6();
    for(i =0; i < len; i++)
        uartComData6.sendBuffer[i] = str[i];
    uartComData6.sendBuffer[len] = 0x0D;//CR
    uartComData6.sendBuffer[len + 1] = 0x0A;//LF
    uartComData6.sendCount = len + 2;
    sendBufferUart6();
    //UART6_Write(str,len);
    return len;
}

//*********** UART 2 GPS *********** 
void clearSendBufferUart2(void);
void clearReceiveBufferUart2(void);
void sendBufferUart2(void);
bool uart2Busy(void);
void uart2_comHandlingDone(void);
//void setUart2Device(int dev);

UART_COM_DATA uartComData2;
uint8_t rxByteU2;
int uart2Mode = UART_GPS;
uint16_t uart2_timeout = 0;
bool uart2_timeoutFlag = false;

void uart2_counters(void)
{
    tempU2Counter++;
    if((uartComData2.state == UART_BUFFER_WAIT_REC) || (uartComData2.state == UART_RECEIVE_BUFFER))
    {
        uart2_timeout++;
        if(uart2_timeout >= 1000)//1000ms
        {
            uart2_timeout = 0;
            uart2_timeoutFlag = true;           
        }
    }
}

void uart2_writeCallback(uintptr_t context)
{
//    U2_EN_Clear();
    uartComData2.writeStatus = true;
}

void uart2_readCallback(uintptr_t context)
{         
    if(uartComData2.receiveCount >  COM_BUFFER_SIZE)
        uartComData2.receiveCount = 0;
    UART2_Read(&rxByteU2, 1);
    asm("NOP");
    asm("NOP");
    uartComData2.receiveBuffer[uartComData2.receiveCount++] = rxByteU2;
    uartComData2.state = UART_RECEIVE_BUFFER;
}

void uart2_comHandlingDone(void)
{
    uartComData2.readStatus = false;
    uartComData2.state = UART_BUFFER_IDLE;
}

void clearSendBufferUart2(void)
{
    memset(uartComData2.sendBuffer,0,COM_BUFFER_SIZE);
    uartComData2.sendCount = 0;
}

void clearReceiveBufferUart2(void)
{
    memset(uartComData2.receiveBuffer,0,COM_BUFFER_SIZE);
    uartComData2.receiveCount = 0;
}

void sendBufferUart2(void)
{
    if(uartComData2.state == UART_BUFFER_IDLE)
        uartComData2.state = UART_SEND_BUFFER;
}

bool uart2Busy(void)
{
    if(uartComData2.state == UART_BUFFER_IDLE)
        return false;
    else
        return true;
}

void uart2_comTasks(void)
{
    uart2Mode = UART_ANEMOMETER;
    uint16_t byteCount = 0;
    switch(uartComData2.state)
    {
        case UART_BUFFER_INIT:
            //*** Test ***
            U2_EN_Clear();
//            setUart2Device(UART_ANEMOMETER);
            initGPSdata();
            uart2Mode == UART_ANEMOMETER;
            UART2_WriteCallbackRegister(uart2_writeCallback, 0);
            UART2_ReadCallbackRegister(uart2_readCallback, 0);
            UART2_Read(&rxByteU2, 1); 
            //*** End Test ***

            uartComData2.writeStatus = false;
            uartComData2.readStatus = false;
            //interCharDelay = 0;
            clearSendBufferUart2();
            clearReceiveBufferUart2();
            uartComData2.state = UART_BUFFER_IDLE;
            break;
        case UART_SEND_BUFFER:
            uart2_timeout = 0;
            if(uart2Mode == UART_ANEMOMETER)
            {
                U2_EN_Set(); 
                while(!U2_EN_Get());
            }
            clearReceiveBufferUart2();
            UART2_Write(uartComData2.sendBuffer, uartComData2.sendCount);
            uartComData2.state = UART_BUFFER_WAIT;
            break;
        case UART_RECEIVE_BUFFER:
            if(uart2Mode == UART_GPS)
            {
                if(uartComData2.receiveBuffer[uartComData2.receiveCount - 1] == 0x0A)//Line Feed
                {
                    uartComData2.state = UART_RECEIVE_PROCESS;
                    uartComData2.readStatus = true;//EOF
                }
            }
            if(uart2Mode == UART_ANEMOMETER)
            {
                byteCount = getExpectedByteCount(uartComData2.receiveBuffer);
                if(byteCount == uartComData2.receiveCount)
                {
                    uartComData2.state = UART_RECEIVE_PROCESS;
                    uartComData2.readStatus = true;//EOF
                }
                if(byteCount > 210)
                {
                    clearReceiveBufferUart2();
                    uartComData2.state = UART_BUFFER_IDLE;
                }
            }
            if(uart2_timeoutFlag)
            {
                uart2_timeoutFlag = false;
                clearReceiveBufferUart2();
                uartComData2.state = UART_BUFFER_IDLE;
            }
            break;
        case UART_RECEIVE_PROCESS:
            if(uart2Mode == UART_GPS)
                parseNMEAstring(uartComData2.receiveBuffer);
            if(uart2Mode == UART_ANEMOMETER)
            {
                parseFrame(uartComData2.receiveBuffer, uartComData2.receiveCount);
                    asm("NOP");
                    asm("NOP");
            }
            clearReceiveBufferUart2();
            uartComData2.state = UART_BUFFER_IDLE;
            break;
        case UART_BUFFER_ERROR:
            break;
        case UART_BUFFER_WAIT:
            if((!UART2_WriteIsBusy())&&(U2STAbits.TRMT))
            {
                uartComData2.state = UART_BUFFER_WAIT_REC;
                if(uart2Mode == UART_ANEMOMETER)
                {
                    U2_EN_Clear(); 
                    while(U2_EN_Get());
                }
            }
            break;
        case UART_BUFFER_WAIT_REC:
            if(uart2_timeoutFlag)
            {
                uart2_timeout = 0;
                uartComData2.state = UART_BUFFER_IDLE;
                uart2_timeoutFlag == false;
            }
            break;
        default:
        case UART_BUFFER_IDLE:           
            //****** Temp switch *****
//            if(tempU2Counter == 10)
//                setUart2Device(UART_ANEMOMETER);
//                
//            if(tempU2Counter == 20)  
//                setUart2Device(UART_GPS);
//            if(tempU2Counter >= 20)
//                tempU2Counter = 0;
             break;
    }
}

void setUart2Device(int dev)
{
//    U2MODEbits.ON = 0;
    /* Disable Interrupts */
//    IEC4CLR = _IEC4_U2EIE_MASK;
//    IEC4CLR = _IEC4_U2RXIE_MASK;
//    IEC4CLR = _IEC4_U2TXIE_MASK;
    
//    UART2_WriteCallbackRegister(NULL, 0);//Deregister
//    UART2_ReadCallbackRegister(NULL, 0);//Deregister
    if(dev == UART_GPS)
    {
        SYSKEY = UNLOCK_KEY_CLEAR;
        SYSKEY = UNLOCK_KEY_1;
        SYSKEY = UNLOCK_KEY_2;
        CFGCONbits.IOLOCK = 0;
        U2RXR = 0b0111;
        RPF2R = 0b0010;
        //uart2Mode = UART_GPS;
    }
    if(dev == UART_ANEMOMETER)
    {
        SYSKEY = UNLOCK_KEY_CLEAR;
        SYSKEY = UNLOCK_KEY_1;
        SYSKEY = UNLOCK_KEY_2;
        CFGCONbits.IOLOCK = 0;
        U2RXR = 0b0110;
        RPB6R = 0b0010;
        uart2Mode = UART_ANEMOMETER;
    }
    CFGCONbits.IOLOCK = 1;
    SYSKEY = UNLOCK_KEY_CLEAR;
    asm("NOP");
    asm("NOP");
    
//    U2MODEbits.ON = 1;
    
//    IEC4SET = _IEC4_U2TXIE_MASK;
//    IEC4SET = _IEC4_U2EIE_MASK;
//    IEC4SET = _IEC4_U2RXIE_MASK;
    
//    U2MODESET = _U2MODE_ON_MASK;
    UART2_Initialize();
    UART2_WriteCallbackRegister(uart2_writeCallback, 0);
    UART2_ReadCallbackRegister(uart2_readCallback, 0);
    int i;
    for(i=0;i<10000;i++);
    asm("NOP");
    asm("NOP");
    UART2_Read(&rxByteU2, 1);
    return;
}

void pollAnenometer(void)
{
    //if((uartComData2.state == UART_BUFFER_IDLE) && (uart2Mode == UART_ANEMOMETER))
    if(uartComData2.state == UART_BUFFER_IDLE)
    {
        uartComData2.sendCount = creatReadIR(uartComData2.sendBuffer, 1, IR_MEAN_WIND_SPEED, 96);
        uartComData2.state = UART_SEND_BUFFER;
    }
}