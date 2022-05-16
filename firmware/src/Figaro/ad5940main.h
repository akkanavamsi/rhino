
#ifndef _AD5940MAIN_H
#define _AD5940MAIN_H

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stddef.h>
#include <stdlib.h>
#include "configuration.h"
//#include "ad5940Lib.h"


typedef enum
{
    AD5940_STATE_INIT = 0,
    AD5940_STATE_RUN,
    AD5940_STATE_ERROR,
} AD5940_STATES;

typedef struct
{
    AD5940_STATES state;
} AD5940_DATA;

void AD5940MAIN_Initialize(void);

void AD5940MAIN_Tasks(void);

#endif /* _AD5940MAIN_H */

/*******************************************************************************
 End of File
 */

