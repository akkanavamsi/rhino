#include "definitions.h"                // SYS function prototypes

#define HUM_ADDR    0x40

typedef struct
{
    float temperature;
    float humidity;
    char userRegister;
} HUMIDITY_DATA;

extern HUMIDITY_DATA humidityData;
