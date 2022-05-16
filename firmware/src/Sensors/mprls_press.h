#include "definitions.h"                // SYS function prototypes

#define MPRLS_ADDR    0x18

#define MPRLS_outMax 15099494
#define MPRLS_outMin 1677722
#define MPRLS_presMax 25 
#define MPRLS_presMin 0  

#define outDifference (MPRLS_outMax - MPRLS_outMin)
#define presDifference (MPRLS_presMax - MPRLS_presMin)

typedef union MPRLS_BYTES {
    
    uint32_t int24Value;
    struct {
        unsigned char byte0 : 8;
        unsigned char byte1 : 8;
        unsigned char byte2 : 8;
        unsigned char byte3 : 8;
    };
} MPRLS_PRESSURE;

typedef enum
{
    MPRLS_STATE_INIT = 0,
    
    MPRLS_STATE_POLL_STATUS,
    MPRLS_STATE_EOC_DONE,            
} MPRLS_STATES;

typedef struct
{
    MPRLS_PRESSURE pressureU24;

    
    bool eoc;
    uint8_t busy;
    uint16_t status;
    
    float pressure;
    MPRLS_STATES state;
    MPRLS_STATES oldState;
    char userRegister;
} MPRLS_DATA;

