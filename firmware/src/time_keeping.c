#include "time_keeping.h"
#include "nmea.h"
#include "mdot.h"

bool rtcc_trigger = false;
TIME_KEEPING_DATA timeData;

struct tm sys_time;
struct tm alarm_time;

void getGPStime(void);
int getDOW(int d, int m, int y);

void rtcc_Callback( uintptr_t context)
{
    rtcc_trigger = true;
}

void timeKeepingTasksInit(void)
{
    timeData.state = RTCC_STATE_INIT;
}

void timeKeepingTasks(void)
{
    switch(timeData.state)
    {
        case RTCC_STATE_INIT:
            RTCC_CallbackRegister(rtcc_Callback, (uintptr_t) NULL);
            timeData.state = RTCC_STATE_SYNC_GPS;
            break;
        case RTCC_STATE_SYNC_GPS:
            if(rmcData.status == 'A')
                timeData.state = RTCC_STATE_SYNC_RTCC;
            break;
        case RTCC_STATE_SYNC_RTCC:
            //getGPStime();
            if(!RTCC_TimeSet(&sys_time))
            {
                timeData.state = RTCC_STATE_ERROR;
                break;
            }
            if(!RTCC_AlarmSet(&alarm_time, RTCC_ALARM_MASK_SS))//every 1 minute
            {
                timeData.state = RTCC_STATE_ERROR;
                break;
            }
            timeData.state = RTCC_STATE_RUN;         
            break;
        case RTCC_STATE_RUN:
            RTCC_TimeGet(&sys_time);
            if(rtcc_trigger == true)//1 minute flag sync to RTCC
            {
                //sendMeasurementGA();
                asm("NOP");
                asm("NOP");
                rtcc_trigger = false;
            }
            break;
        case RTCC_STATE_ERROR:
            break;
        default:
            break;
    }
}

int getDOW(int d, int m, int y)
{
    static int t[] = { 0, 3, 2, 5, 0, 3, 5, 1, 4, 6, 2, 4 };
    y -= m < 3;
    return ( y + y/4 - y/100 + y/400 + t[m-1] + d) % 7;
}

void getGPStime(void)
{    
    sys_time.tm_hour = 0;
    sys_time.tm_min = 0;
    sys_time.tm_sec = 0;

    sys_time.tm_year = 1990;
    sys_time.tm_mon = 1;
    sys_time.tm_mday = 1;
    sys_time.tm_wday = getDOW(1, 1, 1990);
    
//    sys_time.tm_hour = rmcData.hour;
//    sys_time.tm_min = rmcData.minute;
//    sys_time.tm_sec = rmcData.second;
//
//    sys_time.tm_year = rmcData.year;
//    sys_time.tm_mon = rmcData.month;
//    sys_time.tm_mday = rmcData.day;
//    sys_time.tm_wday = getDOW(rmcData.day, rmcData.month, 2000 + rmcData.year);
    
    // Alarm setting 21-04-2022 12:00:00 Thursday, will repeat every 1 minute sync to rtcc
    alarm_time.tm_hour = 00;
    alarm_time.tm_min = 00;
    alarm_time.tm_sec = 00;

    alarm_time.tm_year = 00;
    alarm_time.tm_mon = 01;
    alarm_time.tm_mday = 01;
    alarm_time.tm_wday = 1;
}

void syncGPStime(void)
{
    timeData.state = RTCC_STATE_SYNC_GPS;
}