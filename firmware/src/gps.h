/* 
 * File:   gps.h
 * Author: RFord7
 *
 * Created on 28 January 2022, 12:55
 */

#ifndef GPS_H
#define	GPS_H

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>

typedef struct
{
	uint8_t hour;
	uint8_t minute;
	uint8_t second;
    uint8_t secondBy100;
	uint8_t year;
	uint8_t month;
	uint8_t day;
	double latitude;
	double latitude_dms;
	char latitude_direction;
	double longitude;
	double longitude_dms;
	char longitude_direction;
    int8_t fix_quality;
    uint8_t satellites;
    uint8_t verticle_accuracy;
    uint8_t horizontal_accuracy;
    int16_t altitude;
}GPS_DATA;

void initGPS(void);
void createUBXframe(void);

#endif	/* GPS_H */

