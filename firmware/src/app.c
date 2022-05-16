
#include "app.h"
#include <string.h>
#include "definitions.h" 
#include "communication.h"
#include "mdot.h"
#include "eeprom.h"
#include "Figaro/ad5940main.h"
#include "nmea.h"
#include "time_keeping.h"
#include "auxillary_sensors.h"
#include "messaging.h"
#include "anemometer.h"
#include "console.h"
#include "statistics_buffers.h"

APP_DATA appData;
uint32_t clkFreq,divider;
char latch = false;
int count100ms = 0;
bool flag100ms = false;
int count10ms = 0;
bool flag10ms = false;
int count500ms = 0;
bool flag500ms = false;
int count1sec = 0;
bool flag1sec = false;

extern double gTemp;
extern double gRH;
extern int gCount;

void sampleSensorData(void);

void cyclicCallback(uintptr_t context,uint32_t alarmCount)//Manages cyclic updates
{
    count10ms++;
    if(count10ms >= 10)
    {
        flag10ms = true;
        count10ms = 0;
    }
    count100ms++;
    if(count100ms >= 100)
    {
        count100ms = 0;
        flag100ms = true;
    }
    count500ms++;
    if(count500ms>= 500)
    {
        count500ms = 0;
        flag500ms = true;
    }
    count1sec++;
    if(count1sec >= 1000)
    {
        count1sec = 0;
        flag1sec = true;
    }
}

void initCyclicTImer(void)
{
    clkFreq = TMR2_FrequencyGet();//timer running frequency
    divider = clkFreq / 1000;//calculate the divider needed
    TMR2_PeriodSet(divider);//1ms
    TMR2_CallbackRegister(cyclicCallback,(uintptr_t)NULL);
    TMR2_Start();
}

void APP_Initialize(void)
{
    appData.state = APP_STATE_INIT;
}

void APP_Tasks(void)
{
    switch ( appData.state )
    {
        case APP_STATE_INIT:
        {
            LS_Clear();
            initCyclicTImer();
            communication_init();
            timeKeepingTasksInit();
            mDotInit();
            eepromInit();
            I2C2TASKS_Initialize();
            U5_EN_Set();
            LED2_BL_Set();
            LED2_RD_Set();
            LED2_GR_Set();
            V12_EN_Set();
            FIG_3V3_EN_Set(); 
            FIG_EN_Clear();
            GPS_U_RST_Set();
            AD5940MAIN_Initialize();    
            initSensorAverages();
            appData.state = APP_STATE_SERVICE_TASKS;
            DO1_Set();
            break;
        }
        case APP_STATE_SERVICE_TASKS:
        {
            communication_task();
            timeKeepingTasks();
            mDotTasks();
            AD5940MAIN_Tasks();
            I2C2TASKS_Tasks();
            //eepromTasks();           
            if(flag1sec)//1 second tasks
            {      
                pollAnenometer();
                sampleSensorData();
                mDotCounters();
                char temp[300];
                double tf = figaroPPM;
                double compass = ((float)anemometerData.raw.compass)/10;
                if(rmcData.status== 'A')
                    sprintf(temp, "GPS: A, CH4: %f, Dir: %3.2f, T: %2.2f, RH: %3.2f - %d\n", tf,compass, gTemp, gRH, gCount);
                else
                    sprintf(temp, "GPS: X, CH4: %f, Dir: %3.2f, T: %2.2f, RH: %3.2f - %d\n", tf,compass, gTemp, gRH, gCount);               
                printConsoleStr(temp);
                flag1sec = false;
            }
            if(flag500ms)//500ms tasks
            {
                pollAuxSensors();
                //getAuxSensorAverages(&pres, &tem, &hum);
                flag500ms = false;
            }
            if(flag100ms)//100ms tasks
            {          
                HB_Toggle();
                flag100ms = false;
            }
            if(flag10ms)//10ms tasks
            {
                uart_counters();
                flag10ms = false;
            }             
            break;
            WDT_Clear();
        }
        default:
        {
            break;
        }
    }
}

void sampleSensorData(void)
{
    pushStatSample(&(sensorAverage.ch4), (float)figaroPPM);
    float compass = ((float)anemometerData.raw.compass)/10;
    pushStatSample(&(sensorAverage.compass), compass);
    pushStatSample(&(sensorAverage.humidity), (float)gRH);
    pushStatSample(&(sensorAverage.temperature), (float)gTemp);
    pushWindSample(((float)anemometerData.raw.windSpeed)/10,
                   ((float)anemometerData.raw.windDirection)/100);
}
