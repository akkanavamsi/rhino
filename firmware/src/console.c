#include "console.h"
#include "definitions.h" 
#include <string.h>

void printConsole(float f)
{
    while(!U5STAbits.TRMT);
    char data[64];
    char count = sprintf(data,">%f\r\n",f);
    UART5_Write((uint8_t*)data, count);
    while(!U5STAbits.TRMT);
    asm("NOP");
    asm("NOP");
}

void printConsoleStr(char *data)
{
    while(!U5STAbits.TRMT);
    int count;
    count = strlen(data);
    UART5_Write((uint8_t*)data, count);
    while(!U5STAbits.TRMT);
    asm("NOP");
    asm("NOP");
}

void printConsole2(char *data, int length)
{
    while(!U5STAbits.TRMT);
    UART5_Write((uint8_t*)data, length);
    while(!U5STAbits.TRMT);
    asm("NOP");
    asm("NOP");
}

void createHexString(const unsigned char *src, int len, unsigned char *dest)
{
    static const unsigned char table[] = "0123456789abcdef";

    for (; len > 0; --len)
    {
        unsigned char c = *src++;
        *dest++ = table[c >> 4];
        *dest++ = table[c & 0x0f];
    }
    *dest = 0; 
}

int read(int handle, void *buffer, unsigned int len) { // stub implementation to prevent warning in XC32 v4.00
return 0;
}