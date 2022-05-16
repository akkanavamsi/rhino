/* 
 * File:   statistics_buffers.h
 * Author: RFord7
 *
 * Created on 29 April 2022, 20:02
 */
#ifndef STATISTICS_BUFFERS_H
#define	STATISTICS_BUFFERS_H

#ifdef	__cplusplus
extern "C" {
#endif
    
#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include <string.h>
    
#define STATISTICS_BUFFER_SIZE 120
    
typedef struct
{
    float samples[STATISTICS_BUFFER_SIZE];
    bool full;
    int head;
}STATISTICS_ROLLING_AVE;

typedef struct
{
    STATISTICS_ROLLING_AVE ch4;
    STATISTICS_ROLLING_AVE temperature;
    STATISTICS_ROLLING_AVE humidity;
    STATISTICS_ROLLING_AVE compass;
    STATISTICS_ROLLING_AVE windSpeed; //scalar wind speed average
    STATISTICS_ROLLING_AVE windU; //vector wind average (sin)
    STATISTICS_ROLLING_AVE windV; //vector wind average (cos)
    STATISTICS_ROLLING_AVE windDirSin; //sin wind direction
    STATISTICS_ROLLING_AVE windDirCos; //cos wind direction
}STATISTICS_ROLL_AVE;

//core statistics function
void initStatRollAve(STATISTICS_ROLLING_AVE *ra);
void clearStatRollAve(STATISTICS_ROLLING_AVE *ra);
float getStatRollAve(STATISTICS_ROLLING_AVE *ra);
float getStatRollStdDev(STATISTICS_ROLLING_AVE *ra);
void pushStatSample(STATISTICS_ROLLING_AVE *ra, float value);
void initSensorAverages(void);

extern STATISTICS_ROLL_AVE sensorAverage;

#ifdef	__cplusplus
}
#endif

#endif	/* STATISTICS_BUFFERS_H */


