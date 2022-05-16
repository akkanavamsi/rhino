#include "auxillary_sensors.h"

I2C2TASKS_DATA i2c2tasksData;
uint8_t i2c2_tx_buffer[20];
uint8_t i2c2_rx_buffer[20]; 

int temperature;
unsigned int humidity;

MPRLS_DATA pressureData;
HUMIDITY_DATA humidityData;

ROLLING_AVE pressureRA,temperatureRA,humidityRA;

void waitI2C2(void);
void pressureTasks(void);
float getRollAve(ROLLING_AVE *ra);
void pushSample(ROLLING_AVE *ra, float value);
void initRollAve(ROLLING_AVE *ra);

double gTemp;
double gRH;

void I2C2TASKS_Initialize(void)
{
    i2c2tasksData.state = I2C2TASKS_STATE_INIT;
}

void I2C2TASKS_Tasks(void)
{
    switch(i2c2tasksData.state)
    {
        case I2C2TASKS_STATE_INIT:
            PT_RES_Set();
            initRollAve(&pressureRA);
            initRollAve(&temperatureRA);
            initRollAve(&humidityRA);
            pressureData.state = MPRLS_STATE_INIT; 
            i2c2tasksData.state = I2C2TASKS_STATE_SHT21_CONFIG;
            break;
        case I2C2TASKS_STATE_IDLE:
            break;
        case I2CTASKS_STATE_READ_PRESSURE:
            pressureTasks();
            i2c2tasksData.state = I2C2TASKS_STATE_SHT21_READ_TEMP;
            break;
        case I2C2TASKS_STATE_SHT21_CONFIG:
            waitI2C2();
            i2c2_tx_buffer[0] = 0b11100111;      // READ USER REGISTER
            I2C2_WriteRead(HUM_ADDR, i2c2_tx_buffer, 1, i2c2_rx_buffer, 1);
            waitI2C2();
            i2c2_tx_buffer[1] = i2c2_rx_buffer[0] | 0b10000001; // SET 11BIT
            i2c2_tx_buffer[0] = 0b11100110;      // WRITE USER REGISTER     
            I2C2_Write(HUM_ADDR, i2c2_tx_buffer, 2);
            waitI2C2();
            i2c2_tx_buffer[0] = 0b11100111;      // READ USER REGISTER
            I2C2_WriteRead(HUM_ADDR, i2c2_tx_buffer, 1, i2c2_rx_buffer, 1);
            waitI2C2();
            i2c2tasksData.state = I2C2TASKS_STATE_SHT21_READ_CONFIG;
            break;
        case I2C2TASKS_STATE_SHT21_READ_CONFIG:
            waitI2C2();
            i2c2_tx_buffer[0] = 0b11100111;      // READ USER REGISTER
            I2C2_WriteRead(HUM_ADDR, i2c2_tx_buffer, 1, i2c2_rx_buffer, 1);
            waitI2C2();
            i2c2tasksData.state = I2C2TASKS_STATE_SHT21_READ_TEMP;
            break;
        case I2C2TASKS_STATE_SHT21_READ_TEMP:
            waitI2C2(); 
            i2c2_tx_buffer[0] = 0b11100011;  // Temperature Measurement - NON BLOCK
            I2C2_WriteRead(HUM_ADDR, i2c2_tx_buffer, 1, i2c2_rx_buffer, 3);
            waitI2C2();
            temperature = ((i2c2_rx_buffer[0] << 8) | i2c2_rx_buffer[1]);                 // TYP: 0xA0861C74, 0xA0E9FC73, 0xA06FDC73
            humidityData.temperature = (-46.85 +(175.75*((float)temperature / 65536)));  
            i2c2tasksData.state = I2C2TASKS_STATE_SHT21_READ_HUMID;
            break;
        case I2C2TASKS_STATE_SHT21_READ_HUMID:
            waitI2C2(); 
            i2c2_tx_buffer[0] = 0b11100101;  // Temperature Measurement - NON BLOCK
            I2C2_WriteRead(HUM_ADDR, i2c2_tx_buffer, 1, i2c2_rx_buffer, 3);
            waitI2C2(); 
            humidity = ((i2c2_rx_buffer[0] << 8) | i2c2_rx_buffer[1]);            // TYP: 0x52DE, 0x5176 0x5306
            humidityData.humidity = (-6.0 +(125.0*((float)humidity / 65536)));              
            i2c2tasksData.state = I2C2TASKS_STATE_PROCESS_DATA;
            break;
        case I2C2TASKS_STATE_PROCESS_DATA:
            gTemp = humidityData.temperature;
            gRH = humidityData.humidity;
            pushSample(&pressureRA,pressureData.pressure);
            pushSample(&temperatureRA,humidityData.temperature);
            pushSample(&humidityRA,humidityData.humidity);
            i2c2tasksData.state = I2C2TASKS_STATE_IDLE;
            break;
        default:
            break;
    }
}

void pressureTasks(void)
{
    pressureData.eoc = PT_EOC_Get();       
    switch(pressureData.state)
    {            
        case MPRLS_STATE_INIT:
            waitI2C2();
            i2c2_tx_buffer[0] = 0xAA;      // READ USER REGISTER
            i2c2_tx_buffer[1] = 0x00;      // READ USER REGISTER
            i2c2_tx_buffer[2] = 0x00;      // READ USER REGISTER
            I2C2_Write(MPRLS_ADDR, i2c2_tx_buffer, 3);
            waitI2C2();
            pressureData.state = MPRLS_STATE_POLL_STATUS;          
            break;
        case MPRLS_STATE_POLL_STATUS:
            waitI2C2();
            I2C2_Read(MPRLS_ADDR, i2c2_rx_buffer, 1);
            waitI2C2();
            pressureData.status = (uint16_t)i2c2_rx_buffer[0];
            pressureData.busy = i2c2_rx_buffer[0] & 0b00100000;
            if(pressureData.busy == 0x0)
                pressureData.state = MPRLS_STATE_EOC_DONE;
            break;
        case MPRLS_STATE_EOC_DONE:
            i2c2_rx_buffer[0] = 0x0;      
            i2c2_rx_buffer[1] = 0x0;
            i2c2_rx_buffer[2] = 0x0;
            i2c2_rx_buffer[3] = 0x0;
            waitI2C2();
            I2C2_Read(MPRLS_ADDR, i2c2_rx_buffer, 4);
            waitI2C2();
            I2C2_Read(MPRLS_ADDR, i2c2_rx_buffer, 4);
            pressureData.pressureU24.byte0 = i2c2_rx_buffer[3];
            pressureData.pressureU24.byte1 = i2c2_rx_buffer[2];
            pressureData.pressureU24.byte2 = i2c2_rx_buffer[1];
            pressureData.pressureU24.byte3 = 0x0;
            pressureData.pressure = (float)((pressureData.pressureU24.int24Value - MPRLS_outMin) * MPRLS_presMax);
            pressureData.pressure = pressureData.pressure / (float)outDifference;
            pressureData.state = MPRLS_STATE_INIT;  
            break;
        default:
            break;
    }
}

void waitI2C2()
{
    while(I2C2_IsBusy());
} 

void pollAuxSensors(void)
{
    if(i2c2tasksData.state == I2C2TASKS_STATE_IDLE)
        i2c2tasksData.state = I2CTASKS_STATE_READ_PRESSURE;
}

void initRollAve(ROLLING_AVE *ra)
{
    memset(ra->samples,0, ROLL_BUFFER_SIZE * 4);
    ra->full = false;
    ra->head = 0;
}

float getRollAve(ROLLING_AVE *ra)
{
    float ave = 0;
    int i;
    if(ra->full)
    {
        for(i = 0; i < ROLL_BUFFER_SIZE;i++)
            ave += ra->samples[i];
        ave = ave/ROLL_BUFFER_SIZE;
    }
    else
    {
        for(i = 0; i < ra->head;i++)
            ave += ra->samples[i];
        ave = ave/ra->head;
    }
    return ave;
}

void pushSample(ROLLING_AVE *ra, float value)
{
    if(ra->head >= ROLL_BUFFER_SIZE)
    {
        ra->full = true;
        ra->head = 0;
    }
    ra->samples[ra->head] = value;
    ra->head++;
}

void getAuxSensorAverages(float *pressure, float *temperature, float *humidity)
{
    *pressure = getRollAve(&pressureRA);
    *temperature = getRollAve(&temperatureRA);
    *humidity = getRollAve(&humidityRA);
}

void getAuxSensorAveragesD(double *pressure, double *temperature, double *humidity)
{
    *pressure = getRollAve(&pressureRA);
    *temperature = getRollAve(&temperatureRA);
    *humidity = getRollAve(&humidityRA);
}