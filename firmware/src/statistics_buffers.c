
#include "statistics_buffers.h"
#include <math.h>
STATISTICS_ROLL_AVE sensorAverage;

float getMean(float data[], uint16_t size);
float getStdDev(float data[], uint16_t size);

void initSensorAverages(void)
{
    initStatRollAve(&(sensorAverage.ch4));
    initStatRollAve(&(sensorAverage.compass));
    initStatRollAve(&(sensorAverage.humidity));
    initStatRollAve(&(sensorAverage.temperature));
}

float getMean(float data[], uint16_t size)
{
    float retVal = 0;
    uint16_t i;
    if (size==0) return (0.0F/0.0F); //NAN
    for(i=0;i<size;i++)
        retVal += data[i];
    return retVal/size;
}

float getStdDev(float data[], uint16_t size)
{
    float retVal = 0;
    float diff, mean;
    uint16_t i;
    
    if (size==0) return (0.0F/0.0F); //NAN
    mean = getMean(data, size);
    
    for(i=0;i<size;i++)
    {
        diff = data[i] - mean;
        retVal += diff * diff;
    }
    retVal = retVal/size;
    return (float)(sqrt((double)(retVal)));
}

void initStatRollAve(STATISTICS_ROLLING_AVE *ra)
{
    memset(ra->samples,0, STATISTICS_BUFFER_SIZE * 4);
    ra->full = false;
    ra->head = 0;
}

void clearStatRollAve(STATISTICS_ROLLING_AVE *ra)
{
    initStatRollAve(ra);
}

float getStatRollAve(STATISTICS_ROLLING_AVE *ra)
{
    if(ra->full)
    {
        return getMean(ra->samples,STATISTICS_BUFFER_SIZE);
    }
    else
    {
        return getMean(ra->samples,ra->head);
    }
}

float getStatRollStdDev(STATISTICS_ROLLING_AVE *ra)
{
    if(ra->full)
    {
        return getStdDev(ra->samples,STATISTICS_BUFFER_SIZE);
    }
    else
    {
        return getStdDev(ra->samples,ra->head);
    }
}

void pushStatSample(STATISTICS_ROLLING_AVE *ra, float value)
{
    if(ra->head >= STATISTICS_BUFFER_SIZE)
    {
        ra->full = true;
        ra->head = 0;
    }
    ra->samples[ra->head] = value;
    ra->head++;
}
