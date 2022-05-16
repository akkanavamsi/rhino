/* 
 * File:   time_keeping.h
 * Author: RFord7
 *
 * Created on 21 April 2022, 12:17
 */

#ifndef TIME_KEEPING_H
#define	TIME_KEEPING_H

#ifdef	__cplusplus
extern "C" {
#endif

    #include <stddef.h>
    #include <stdlib.h>
    #include <stdint.h>
    #include <stdbool.h>
    #include "definitions.h" 
    
    typedef enum
    {
        RTCC_STATE_INIT,
        RTCC_STATE_SYNC_GPS,
        RTCC_STATE_SYNC_RTCC,
        RTCC_STATE_RUN,
        RTCC_STATE_ERROR,
    }RTCC_STATE;
    
    typedef struct
    {
        RTCC_STATE state;
    }TIME_KEEPING_DATA;
    
    void timeKeepingTasksInit(void);
    void timeKeepingTasks(void);
    void syncGPStime(void);
    
    extern struct tm sys_time;
   
#ifdef	__cplusplus
}
#endif

#endif	/* TIME_KEEPING_H */

