#include "nmea.h"
#include "communication.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "console.h"

int hexchar2int(char c);
int hex2int(char *c);
int checksumOK(char *string);
int parseCDLstring(char *string, char **fields, int max_fields);
void createUBXframe(void);

GPS_RMC rmcData;

extern UART_COM_DATA uartComData2;

char *strA[100];
char* tok;

void initGPSdata(void)
{
    rmcData.is_valid = 0;
    rmcData.hour = 0;
    rmcData.minute = 0;
    rmcData.second = 0;
    rmcData.year = 0;
    rmcData.month = 0;
    rmcData.day = 0;
    rmcData.latitude = 0;
    rmcData.latitude_dms = 0;
    rmcData.latitude_direction = 0;
    rmcData.longitude = 0;
    rmcData.longitude_dms = 0;
    rmcData.longitude_direction = 0;
    createUBXframe();
}

int parseNMEAstring(char *str)
{
	char *field[50];
    char *test = &(uartComData2.receiveBuffer[3]);
    if (strncmp(test, "RMC", 3) == 0)
    {
        char *test2 = &(uartComData2.receiveBuffer[0]);
        //printConsoleStr(test2);
        parseCDLstring(test2, field, 20);
        char *rest;
        uint32_t utc = strtol(field[1], &rest, 10);
        rmcData.hour = utc / 10000;
        rmcData.minute = utc / 100 - rmcData.hour * 100;
        rmcData.second = utc - rmcData.hour * 10000 - rmcData.minute * 100;
        uint32_t date = strtol(field[9], &rest, 10);
        rmcData.day = date / 10000;
        rmcData.month = date / 100 - rmcData.day * 100;
        rmcData.year = date - rmcData.day * 10000 - rmcData.month * 100;
        rmcData.status = field[2][0];
        rmcData.latitude_dms = strtod(field[3], &rest);
        rmcData.latitude_direction = field[4][0];
        char *lat = field[3];
        rmcData.latitude = (lat[0] - '0') * 10 + lat[1] - '0';
        rmcData.latitude += strtod(lat + 2, &rest) / 60;
        if (rmcData.latitude_direction == 'S') 
            rmcData.latitude *= -1;
        rmcData.longitude_dms = strtod(field[5], &rest);
        rmcData.longitude_direction = field[6][0];
        char *lon = field[5];
        rmcData.longitude = (lon[0] - '0') * 100 + (lon[1] - '0') * 10 + lon[2] - '0';
        rmcData.longitude += strtod(lon + 3, &rest) / 60;
        if (rmcData.longitude_direction == 'W')
            rmcData.longitude *= -1;
        }
    asm("NOP");
    asm("NOP");
    return -1;
}

int checksumOK(char *string)
{
	char *checksum_str;
	int checksum;
	unsigned char calculated_checksum = 0;

	// Checksum is postcede by *
	checksum_str = strchr(string, '*');
	if (checksum_str != NULL)
    {
		// Remove checksum from string
		*checksum_str = '\0';
		// Calculate checksum, starting after $ (i = 1)
		for (int i = 1; i < strlen(string); i++) 
			calculated_checksum = calculated_checksum ^ string[i];
		checksum = hex2int((char *)checksum_str+1);
		if (checksum == calculated_checksum)
			return 1;
	} 
    else
		return 0;
	return 0;
}

int hexchar2int(char c)
{
    if (c >= '0' && c <= '9')
        return c - '0';
    if (c >= 'A' && c <= 'F')
        return c - 'A' + 10;
    if (c >= 'a' && c <= 'f')
        return c - 'a' + 10;
    return -1;
}

int hex2int(char *c)
{
	int value;
	value = hexchar2int(c[0]);
	value = value << 4;
	value += hexchar2int(c[1]);
	return value;
}

int parseCDLstring(char *string, char **fields, int max_fields)
{
	int i = 0;
	fields[i++] = string;
	while ((i < max_fields) && NULL != (string = strchr(string, ','))) 
    {
		*string = '\0';
		fields[i++] = ++string;
	}
	return --i;
}

void createUBXframe(void)
{
    uartComData2.sendBuffer[0] = 0xB5;
    uartComData2.sendBuffer[1] = 0x62;
    uartComData2.sendBuffer[2] = 0x06;
    uartComData2.sendBuffer[3] = 0x01;
    
    uartComData2.sendBuffer[4] = 0x08;
    uartComData2.sendBuffer[5] = 0x00;
    uartComData2.sendBuffer[6] = 0xF1;
    uartComData2.sendBuffer[7] = 0x00;
    uartComData2.sendBuffer[8] = 0x00;
    uartComData2.sendBuffer[9] = 0x00;
    uartComData2.sendBuffer[10] = 0x00;
    uartComData2.sendBuffer[11] = 0x01;
    uartComData2.sendBuffer[12] = 0x00;
    uartComData2.sendBuffer[13] = 0x00;
    uint8_t ck_a = 0, ck_b = 0;
    int i;
    int y = 12;
    for(i=0;i<y;i++)
    {
        ck_a = ck_a + uartComData2.sendBuffer[i + 2];
        ck_b = ck_a +ck_b;
    }   
    uartComData2.sendBuffer[14] = ck_a;
    uartComData2.sendBuffer[15] = ck_b;
    uartComData2.sendCount = 16;
}