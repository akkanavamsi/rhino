
#ifndef _APP_H
#define _APP_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>
#include "configuration.h"

#define COM_REC_BUFFER_SIZE 256

extern double figaroPPM;

typedef enum
{
    APP_STATE_INIT=0,
    APP_STATE_SERVICE_TASKS,
} APP_STATES;

typedef struct
{
    char receiveBuffer[COM_REC_BUFFER_SIZE];
    bool messageReceived;
    uint16_t head;
}COMM_DATA;

typedef struct
{
    APP_STATES state;
    COMM_DATA comm;
} APP_DATA;

void APP_Initialize(void);

void APP_Tasks(void);

#endif /* _APP_H */

