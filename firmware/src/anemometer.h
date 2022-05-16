/* 
 * File:   anemometer.h
 * Author: RFord7
 *
 * Created on 26 April 2022, 14:09
 */

#ifndef ANEMOMETER_H
#define	ANEMOMETER_H

#ifdef	__cplusplus
extern "C" {
#endif
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "statistics_buffers.h"
    
    #define IR_MEAN_WIND_SPEED 35001
    #define IR_MEAN_WIND_DIRECTION 35003
    #define IR_PRECIPITATION 35035
    #define IR_COMPASS 35081
    #define IR_RADIATION 35091

    #define INDEX_MEAN_WIND_SPEED 0
    #define INDEX_MEAN_WIND_DIRECTION 4
    #define INDEX_INTERNAL_TEMP 12
    #define INDEX_PRECIPITATION 68
    #define INDEX_COMPASS 160
    #define INDEX_RADIATION 180

    typedef struct
    {
//        uint16_t VAvgWS;
//        uint16_t VAvgWD;
//        uint16_t StdDevWD;
//        uint16_t Msr;
//        uint16_t StdDevSR;
//        uint16_t Prec;
//        uint16_t Cmp;
//        uint16_t SAvgWS ;
//        uint16_t StdDevWS;
//        int8_t AvgWDSin;
//        int8_t AvgWDCos;
        struct
        {
            uint32_t windSpeed;
            uint32_t windDirection;
            uint32_t internalTemperature;
            uint32_t precipitation;
            uint32_t compass;
            int32_t radiation;
        }raw;
        
    }ANEMOMETER_DATA;

    typedef struct
    {
        STATISTICS_ROLLING_AVE windSpeed; //scalar wind speed average
        STATISTICS_ROLLING_AVE windU; //vector wind average (sin)
        STATISTICS_ROLLING_AVE windV; //vector wind average (cos)
        STATISTICS_ROLLING_AVE windDirSin; //sin wind direction
        STATISTICS_ROLLING_AVE windDirCos; //cos wind direction
    } ANEMOMETER_ROLL_AVE;
    
    extern ANEMOMETER_DATA anemometerData;
    extern ANEMOMETER_ROLL_AVE anemometerAverage;
    
    void initAnemometer(void);
    float convert_cardinal_to_polar(float windDir_cardinal);
    void clearAnemometerSamples(void);
    void pushAnemometerSamples(void);
    void pushWindSamplePolar(float windDir, float windSpeed);
    void pushWindSampleCardinal(float windDir, float windSpeed);
    void getWindVectorMean(float* windSpeed_vector_avg, 
                           float* windDir_vector_avg);
    void getWindVectorStdDev(float* windDir_stdDev, 
                           float* windDir_sin_avg, 
                           float* windDir_cos_avg);
    void getWindSpeedScalarAvgStdDev(float* windSpeed_avg, 
                           float* windSpeed_stdev);    
    void prepareAnemometerData(void);

#ifdef	__cplusplus
}
#endif

#endif	/* ANEMOMETER_H */

