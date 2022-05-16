#include "gps.h"
#include "communication.h"


GPS_DATA gpsData;

int parseCSVstring(char *string, char **fields, int max_fields)
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

void initGPS(void)
{
    gpsData.hour = 0;
    gpsData.minute = 0;
    gpsData.second = 0;
    gpsData.secondBy100 = 0;
    gpsData.year = 0;
    gpsData.month = 0;
    gpsData.day = 0;
    gpsData.latitude = 0;
    gpsData.latitude_dms = 0;
    gpsData.latitude_direction = 0;
    gpsData.longitude = 0;
    gpsData.longitude_dms = 0;
    gpsData.longitude_direction = 0;
    gpsData.altitude = 0;
    gpsData.fix_quality = 0;
    gpsData.horizontal_accuracy = 0;
    gpsData.verticle_accuracy = 0;
    gpsData.satellites = 0;
}

void parsePUBX(void)
{
//    char *field[50];
//    char *test = &(uartComData3.receiveBuffer[3]);
//    char seps[]   = ",";
//    int i = 0; 
//    if (strncmp(test, "PUBX", 4) == 0)
//    {
//        test = &(uartComData3.receiveBuffer[0]);
//        parseCSVstring(test, field, 50);
//        
//        char *rest;
//        uint32_t utc = strtol(field[2], &rest, 10);
//        rmcData.hour = utc / 10000;
//        rmcData.minute = utc / 100 - rmcData.hour * 100;
//        rmcData.second = utc - rmcData.hour * 10000 - rmcData.minute * 100;
//        uint32_t date = strtol(field[9], &rest, 10);
//        rmcData.day = date / 10000;
//        rmcData.month = date / 100 - rmcData.day * 100;
//        rmcData.year = date - rmcData.day * 10000 - rmcData.month * 100;
//        
//        rmcData.latitude_dms = strtod(field[3], &rest);
//        rmcData.latitude_direction = field[4];
//        char *lat = field[3];
//        rmcData.latitude = (lat[0] - '0') * 10 + lat[1] - '0';
//        rmcData.latitude += strtod(lat + 2, &rest) / 60;
//        if (field[4] == 'S') 
//            rmcData.latitude *= -1;
//        
//        rmcData.longitude_dms = strtod(field[5], &rest);
//        rmcData.longitude_direction = field[6];
//        char *lon = field[5];
//        rmcData.longitude = (lon[0] - '0') * 100 + (lon[1] - '0') * 10 + lon[2] - '0';
//        rmcData.longitude += strtod(lon + 3, &rest) / 60;
//	if (field[6]== 'W')
//		rmcData.longitude *= -1;
//        
//        
//    
//        for(i=0;i<10;i++)
//        {
//            UART3_Write( field[i], strlen(field[i]) );
//            while(UART3_WriteIsBusy());
//            UART3_Write( "\n\r",2 );
//            while(UART3_WriteIsBusy());
//        }
//    }
}

void createUBXframe(void)
{
    uartComData3.sendBuffer[0] = 0xB5;
    uartComData3.sendBuffer[1] = 0x62;
    uartComData3.sendBuffer[2] = 0x06;
    uartComData3.sendBuffer[3] = 0x01;
    
    uartComData3.sendBuffer[4] = 0x08;
    uartComData3.sendBuffer[5] = 0x00;
    uartComData3.sendBuffer[6] = 0xF1;
    uartComData3.sendBuffer[7] = 0x00;
    uartComData3.sendBuffer[8] = 0x00;
    uartComData3.sendBuffer[9] = 0x00;
    uartComData3.sendBuffer[10] = 0x00;
    uartComData3.sendBuffer[11] = 0x01;
    uartComData3.sendBuffer[12] = 0x00;
    uartComData3.sendBuffer[13] = 0x00;
    uint8_t ck_a = 0, ck_b = 0;
    int i;
    int y = 12;
    for(i=0;i<y;i++)
    {
        ck_a = ck_a + uartComData3.sendBuffer[i + 2];
        ck_b = ck_a +ck_b;
    }
    uartComData3.sendBuffer[14] = ck_a;
    uartComData3.sendBuffer[15] = ck_b;
}