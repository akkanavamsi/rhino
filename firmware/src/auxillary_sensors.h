/* 
 * File:   auxillary_sensors.h
 * Author: RFord7
 *
 * Created on 22 April 2022, 10:22
 */

#ifndef AUXILLARY_SENSORS_H
#define	AUXILLARY_SENSORS_H

#ifdef	__cplusplus
extern "C" {
#endif
    
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include "Sensors/mprls_press.h"
#include "Sensors/sht21.h"
    
#define ROLL_BUFFER_SIZE 120

typedef enum
{
    I2C2TASKS_STATE_INIT=0,
    I2C2TASKS_STATE_IDLE,
    I2CTASKS_STATE_READ_PRESSURE,
    I2C2TASKS_STATE_SHT21_CONFIG,
    I2C2TASKS_STATE_SHT21_READ_CONFIG,
    I2C2TASKS_STATE_SHT21_READ_TEMP,
    I2C2TASKS_STATE_SHT21_READ_HUMID,  
    I2C2TASKS_STATE_PROCESS_DATA, 
} I2C2TASKS_STATES;

typedef struct
{
    I2C2TASKS_STATES state;
} I2C2TASKS_DATA;

typedef struct
{
    float samples[ROLL_BUFFER_SIZE];
    bool full;
    int head;
}ROLLING_AVE;

void I2C2TASKS_Initialize(void);
void I2C2TASKS_Tasks(void);
void pollAuxSensors(void);
void getAuxSensorAverages(float *pressure, float *temperature, float *humidity);
void getAuxSensorAveragesD(double *pressure, double *temperature, double *humidity);

#ifdef	__cplusplus
}
#endif

#endif	/* AUXILLARY_SENSORS_H */

