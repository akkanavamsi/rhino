#ifndef NMEA_H
#define	NMEA_H

#include <stdint.h>

typedef struct
{
	uint8_t is_valid;
	char format[6];
	uint8_t hour;
	uint8_t minute;
	uint8_t second;
    uint8_t secondBy100;
	uint8_t year;
	uint8_t month;
	uint8_t day;
    char status;
	double latitude;
	double latitude_dms;
	char latitude_direction;
	double longitude;
	double longitude_dms;
	char longitude_direction;
} GPS_RMC;

extern GPS_RMC rmcData;

int parseNMEAstring(char *str);
void initGPSdata(void);

#endif	/* NMEA_H */

