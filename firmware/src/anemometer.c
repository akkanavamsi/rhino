#include "anemometer.h"
#include <stdint.h>
#include <math.h>
#include "statistics_buffers.h"

ANEMOMETER_DATA anemometerData;
ANEMOMETER_ROLL_AVE anemometerAverage;

#define degToRad(angleInDegrees) ((angleInDegrees) * M_PI / 180.0)
#define radToDeg(angleInRadians) ((angleInRadians) * 180.0 / M_PI)
//*** Scalar Functions ***

void initAnemometer(void)
{
    clearAnemometerSamples(); 
}

void pushAnemometerSamples(void)
{
    float windSpeed = ((float)(anemometerData.raw.windSpeed))/10.0;
    float windDirection = ((float)(anemometerData.raw.windDirection))/100.0;
    pushWindSampleCardinal(windSpeed, windDirection);
}

void pushWindSamplePolar(float windDir, float windSpeed)
{
    windDir=degToRad(windDir);
    float wdCos, wdSin;
    wdCos = cos(windDir);
    wdSin = sin(windDir);
    pushStatSample(&(anemometerAverage.windV), windSpeed*wdCos);
    pushStatSample(&(anemometerAverage.windU), windSpeed*wdSin);
    pushStatSample(&(anemometerAverage.windDirSin), wdSin);
    pushStatSample(&(anemometerAverage.windDirCos), wdCos);
    pushStatSample(&(anemometerAverage.windSpeed), windSpeed);
}

void pushWindSampleCardinal(float windDir, float windSpeed)
{
    windDir = convert_cardinal_to_polar(windDir);
    pushWindSamplePolar(windDir, windSpeed);
}

void getWindVectorMean(float* windSpeed_vector_avg, 
                       float* windDir_vector_avg)
{
    float u_avg, v_avg;
    u_avg = getStatRollAve(&(anemometerAverage.windU));
    v_avg = getStatRollAve(&(anemometerAverage.windV));
    *windSpeed_vector_avg = (float)(sqrt((double)(u_avg*u_avg + v_avg*v_avg)));
    *windDir_vector_avg = radToDeg((float)(atan2((double)u_avg,(double)v_avg)));
}

void getWindVectorStdDev(float* windDir_stdDev, 
                       float* windDir_sin_avg, float* windDir_cos_avg)
{    
    *windDir_sin_avg = getStatRollAve(&(anemometerAverage.windDirSin));
    *windDir_cos_avg = getStatRollAve(&(anemometerAverage.windDirCos));
    double R = sqrt((*windDir_sin_avg)*(*windDir_sin_avg) 
                    + (*windDir_cos_avg)*(*windDir_cos_avg));
    if (R>1.0) R=1.0;
    *windDir_stdDev = radToDeg((float)sqrt(2.0*(1.0-R)));
}

void prepareAnemometerData(void)
{

}

void clearAnemometerSamples(void)
{
    initStatRollAve(&(anemometerAverage.windSpeed));
    initStatRollAve(&(anemometerAverage.windU));
    initStatRollAve(&(anemometerAverage.windV));
    initStatRollAve(&(anemometerAverage.windDirSin));
    initStatRollAve(&(anemometerAverage.windDirCos));    
}

void getWindSpeedScalarAvgStdDev(float* windSpeed_avg, 
                        float* windSpeed_stdev)
{
    *windSpeed_avg = getStatRollAve(&(anemometerAverage.windSpeed));
    *windSpeed_stdev = getStatRollStdDev(&(anemometerAverage.windSpeed));
}

float convert_cardinal_to_polar(float windDir_cardinal)
{
    float polar = fmod(-(windDir_cardinal + 90),360);
    if (polar<0) polar+=360.0;
    return polar;
}
