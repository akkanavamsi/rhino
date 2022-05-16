#include "messaging.h"
#include "nmea.h"
#include "time_keeping.h"
#include "auxillary_sensors.h"
#include "statistics_buffers.h"
#include "anemometer.h"

MSG_MEASUREMENT_GA measurement;
MSG_PARAMUPD_GA parameterUpdate;
MSG_FRMWUPD_GA firmwareUpdate;
MSG_CMSGDEV_GA commisioning;
MSG_DECMSGDEV_GA decommission;

extern double figaroPPM;

//Test
int gCount = 0;

void initPayload(void)
{
    measurement.Alt = 1005;
    measurement.AvgWDCos = 7;
    measurement.AvgWDSin = 4;
    measurement.BattV = 720;
    measurement.Cmp = 127;
    measurement.Curr = 120;
    measurement.Fq = 12;
    measurement.GasC = 3745;
    measurement.Hacc = 134;
    measurement.Lat = 4717.11399;
    measurement.Lgt = 00833.91590;
    measurement.MHum = 50;
    measurement.MPress = 101;
    measurement.MTemp = 250;
    measurement.Msr = 123;
    measurement.Ns = 8;
    measurement.Prec = 1345;
    measurement.SAvgWS = 13485;
    measurement.SpV = 2367;
    measurement.StdDevG = 129;
    measurement.StdDevSR = 237;
    measurement.StdDevWD = 45;
    measurement.StdDevWS = 89;
    measurement.Tilt = 127;
    measurement.Tsrc = 1;
    measurement.UnitStatus = 1234;
    measurement.VAvgWD = 2765;
    measurement.VAvgWS = 3456;
    measurement.Vacc = 12;
    measurement.Volt = 7200;
    measurement.header.type = 1;
    measurement.header.version = 1;
    measurement.header.timestamp = 63070000;
}

void createMeasurementGA(void)
{
    float P,T,RH;
    getAuxSensorAverages(&P, &T, &RH);
    measurement.Alt = 1005;   
    measurement.BattV = 720;
    double cmp = (double)getStatRollAve(&(sensorAverage.compass));
    measurement.Cmp = (uint16_t)(cmp * 100);
    measurement.Curr = 120;
    measurement.Fq = 12;
    float ppm = getStatRollAve(&(sensorAverage.ch4));
    measurement.GasC = (ppm * 10);
    measurement.Hacc = 134;
    measurement.Lat = rmcData.latitude;
    measurement.Lgt = rmcData.longitude;
    measurement.MHum = (uint8_t)(RH * 2);
    measurement.MPress = (uint16_t)(P * 1000);
    measurement.MTemp = (uint16_t)(T * 100);
    measurement.Msr = 123;
    measurement.Ns = 8;
    
    float precip = (float)anemometerData.raw.precipitation;
    measurement.Prec = (uint16_t)(precip/10);
    measurement.SpV = 2367;
    float stdevGas = getStatRollStdDev(&(sensorAverage.ch4));
    measurement.StdDevG = (uint16_t)stdevGas * 10;
    measurement.StdDevSR = 4456;

    float wdSin,wdCos,wdStdDev, wdVectorAvg, wsVectorAvg;
    getWindVectorMean(&wsVectorAvg,&wdVectorAvg);
    getWindVectorStdDev(&wdStdDev,&wdSin,&wdCos);
    measurement.AvgWDCos = (int8_t)(wdCos*100);
    measurement.AvgWDSin = (int8_t)(wdSin*100);
    float wsS = getStatRollAve(&(sensorAverage.windSpeed));
    measurement.SAvgWS = (uint16_t)(wsS * 100);    
    measurement.StdDevWD = (uint16_t)(wdStdDev * 100);
    float stdevwsS = getStatRollStdDev(&(sensorAverage.windSpeed));
    measurement.StdDevWS = (uint16_t)(stdevwsS * 100);
    measurement.Tilt = gCount++;
    measurement.Tsrc = 1;
    measurement.UnitStatus = 1234;
    measurement.VAvgWD = (uint16_t)(wdVectorAvg * 100);
    measurement.VAvgWS = (uint16_t)(wsVectorAvg * 100);
    measurement.Vacc = 12;
    measurement.Volt = 7200;
    measurement.header.type = MESSAGE_TYPE_MEAS_GA;
    measurement.header.version = MEAS_GA_VERSION;
    
    int year = sys_time.tm_year;
    int month = sys_time.tm_mon;
    int day = sys_time.tm_mday;
    int hour = sys_time.tm_hour;
    int min = sys_time.tm_min;
    int sec = sys_time.tm_sec;
    
    measurement.header.timestamp = unixTimestamp(year, month, day, hour, min, sec);// - 1577836800;//offset Jan 2020
    asm("NOP");
    asm("NOP");
}

uint16_t measurement2Bytes(uint8_t * data)
{
    createMeasurementGA();
    DOUBLE_2_BYTES Double2Bytes;
    data[0] = measurement.header.type;
    data[1] = measurement.header.version;
    data[2] = (measurement.header.timestamp >> 24) & 0xFF;
    data[3] = (measurement.header.timestamp >> 16) & 0xFF;
    data[4] = (measurement.header.timestamp >> 8) & 0xFF;
    data[5] = (measurement.header.timestamp) & 0xFF;
    data[6] = (measurement.GasC >> 8) & 0xFF;
    data[7] = (measurement.GasC) & 0xFF;
    data[8] = (measurement.StdDevG >> 8) & 0xFF;
    data[9] = (measurement.StdDevG) & 0xFF;
    data[10] = (measurement.VAvgWS >> 8) & 0xFF;
    data[11] = (measurement.VAvgWS) & 0xFF;
    data[12] = (measurement.VAvgWD >> 8) & 0xFF;
    data[13] = (measurement.VAvgWD) & 0xFF;
    data[14] = (measurement.StdDevWD>> 8) & 0xFF;
    data[15] = (measurement.StdDevWD) & 0xFF;
    data[16] = (measurement.Msr>> 8) & 0xFF;
    data[17] = (measurement.Msr) & 0xFF;
    data[18] = (measurement.StdDevSR>> 8) & 0xFF;
    data[19] = (measurement.StdDevSR) & 0xFF;
    data[20] = (measurement.Prec>> 8) & 0xFF;
    data[21] = (measurement.Prec) & 0xFF;
    data[22] = (measurement.Cmp>> 8) & 0xFF;
    data[23] = (measurement.Cmp) & 0xFF;
    data[24] = (measurement.SAvgWS>> 8) & 0xFF;
    data[25] = (measurement.SAvgWS) & 0xFF;
    data[26] = (measurement.StdDevWS>> 8) & 0xFF;
    data[27] = (measurement.StdDevWS) & 0xFF;
    data[28] = measurement.AvgWDSin;
    data[29] = measurement.AvgWDCos;
    Double2Bytes.value = measurement.Lat;
    data[30] = Double2Bytes.bytes[7];
    data[31] = Double2Bytes.bytes[6];
    data[32] = Double2Bytes.bytes[5];
    data[33] = Double2Bytes.bytes[4];
    data[34] = Double2Bytes.bytes[3];
    data[35] = Double2Bytes.bytes[2];
    data[36] = Double2Bytes.bytes[1];
    data[37] = Double2Bytes.bytes[0];
    Double2Bytes.value = measurement.Lgt;
    data[38] = Double2Bytes.bytes[7];
    data[39] = Double2Bytes.bytes[6];
    data[40] = Double2Bytes.bytes[5];
    data[41] = Double2Bytes.bytes[4];
    data[42] = Double2Bytes.bytes[3];
    data[43] = Double2Bytes.bytes[2];
    data[44] = Double2Bytes.bytes[1];
    data[45] = Double2Bytes.bytes[0];
    data[46] = (measurement.Alt>> 8) & 0xFF;
    data[47] = (measurement.Alt) & 0xFF;
    data[48] = measurement.Fq;
    data[49] = measurement.Ns;
    data[50] = measurement.Hacc;
    data[51] = measurement.Vacc;
    data[52] = measurement.Tsrc;
    data[53] = (measurement.Tilt>> 8) & 0xFF;
    data[54] = (measurement.Tilt) & 0xFF;
    data[55] = (measurement.BattV>> 8) & 0xFF;
    data[56] = (measurement.BattV) & 0xFF;
    data[57] = (measurement.SpV>> 8) & 0xFF;
    data[58] = (measurement.SpV) & 0xFF;
    data[59] = (measurement.UnitStatus >> 24) & 0xFF;
    data[60] = (measurement.UnitStatus >> 16) & 0xFF;
    data[61] = (measurement.UnitStatus >> 8) & 0xFF;
    data[62] = (measurement.UnitStatus) & 0xFF;
    data[63] = (measurement.Curr>> 8) & 0xFF;
    data[64] = (measurement.Curr) & 0xFF;
    data[65] = (measurement.Volt>> 8) & 0xFF;
    data[66] = (measurement.Volt) & 0xFF;
    data[67] = (measurement.MTemp>> 8) & 0xFF;
    data[68] = (measurement.MTemp) & 0xFF;
    data[69] = (measurement.MPress>> 8) & 0xFF;
    data[70] = (measurement.MPress) & 0xFF;
    data[71] = measurement.MHum;
    return MEASUREMENT_GA_SIZE + LORA_HEADER_SIZE;
}

uint16_t parameterUpdate2Bytes(uint8_t * data)
{
    DOUBLE_2_BYTES Double2Bytes;
    data[0] = parameterUpdate.header.type;
    data[1] = parameterUpdate.header.version;
    data[2] = (parameterUpdate.header.timestamp >> 24) & 0xF;
    data[3] = (parameterUpdate.header.timestamp >> 16) & 0xF;
    data[4] = (parameterUpdate.header.timestamp >> 8) & 0xF;
    data[5] = (parameterUpdate.header.timestamp) & 0xF;
    data[6] = (parameterUpdate.ParamN >> 8) & 0xF;
    data[7] = (parameterUpdate.ParamN) & 0xF;
    Double2Bytes.value = parameterUpdate.Param;
    data[8] = Double2Bytes.bytes[7];
    data[9] = Double2Bytes.bytes[6];
    data[10] = Double2Bytes.bytes[5];
    data[11] = Double2Bytes.bytes[4];
    data[12] = Double2Bytes.bytes[3];
    data[13] = Double2Bytes.bytes[2];
    data[14] = Double2Bytes.bytes[1];
    data[15] = Double2Bytes.bytes[0];
    data[16] = parameterUpdate.Status;
    return MSG_PARAMUPD_GA_SIZE + LORA_HEADER_SIZE;
}

uint16_t firmwareUpdate2Bytes(uint8_t * data)
{
    data[0] = firmwareUpdate.header.type;
    data[1] = firmwareUpdate.header.version;
    data[2] = (firmwareUpdate.header.timestamp >> 24) & 0xF;
    data[3] = (firmwareUpdate.header.timestamp >> 16) & 0xF;
    data[4] = (firmwareUpdate.header.timestamp >> 8) & 0xF;
    data[5] = (firmwareUpdate.header.timestamp) & 0xF;
    data[12] = firmwareUpdate.Status;
    return MSG_FRMWUPD_GA_SIZE + LORA_HEADER_SIZE;
}

uint16_t commisioning2Bytes(uint8_t * data)
{
    DOUBLE_2_BYTES Double2Bytes;
    data[0] = commisioning.header.type;
    data[1] = commisioning.header.version;
    data[2] = (commisioning.header.timestamp >> 24) & 0xF;
    data[3] = (commisioning.header.timestamp >> 16) & 0xF;
    data[4] = (commisioning.header.timestamp >> 8) & 0xF;
    data[5] = (commisioning.header.timestamp) & 0xF;
    data[6] = (commisioning.FrmwGV>> 8) & 0xF;
    data[7] = (commisioning.FrmwGV) & 0xF;
    data[8] = (commisioning.BootGV>> 8) & 0xF;
    data[9] = (commisioning.BootGV) & 0xF;
    data[10] = (commisioning.FrmwWV>> 8) & 0xF;
    data[11] = (commisioning.FrmwWV) & 0xF;
    data[12] = (commisioning.FrmwrMV>> 8) & 0xF;
    data[13] = (commisioning.FrmwrMV) & 0xF;
    data[14] = commisioning.Gen;
    data[15] = commisioning.Utype;
    data[16] = commisioning.R;
    data[17] = (commisioning.Tno>> 8) & 0xF;
    data[18] = (commisioning.Tno) & 0xF;
    Double2Bytes.value = commisioning.Lat;
    data[19] = Double2Bytes.bytes[7];
    data[20] = Double2Bytes.bytes[6];
    data[21] = Double2Bytes.bytes[5];
    data[22] = Double2Bytes.bytes[4];
    data[23] = Double2Bytes.bytes[3];
    data[24] = Double2Bytes.bytes[2];
    data[25] = Double2Bytes.bytes[1];
    data[26] = Double2Bytes.bytes[0];
    Double2Bytes.value = commisioning.Lgt;
    data[27] = Double2Bytes.bytes[7];
    data[28] = Double2Bytes.bytes[6];
    data[29] = Double2Bytes.bytes[5];
    data[30] = Double2Bytes.bytes[4];
    data[31] = Double2Bytes.bytes[3];
    data[32] = Double2Bytes.bytes[2];
    data[33] = Double2Bytes.bytes[1];
    data[34] = Double2Bytes.bytes[0];
    return MSG_CMSGDEV_GA_SIZE + LORA_HEADER_SIZE;
}

uint16_t decommission2Bytes(uint8_t * data)
{
    data[0] = decommission.header.type;
    data[1] = decommission.header.version;
    data[2] = (decommission.header.timestamp >> 24) & 0xF;
    data[3] = (decommission.header.timestamp >> 16) & 0xF;
    data[4] = (decommission.header.timestamp >> 8) & 0xF;
    data[5] = (decommission.header.timestamp) & 0xF;
    data[6] = (decommission.header.timestamp) & 0xF;
    data[7] = decommission.Status;
    return MSG_DECMSGDEV_GA_SIZE + LORA_HEADER_SIZE;
}

unsigned long unixTimestamp(int year, int month, int day, int hour, int min, int sec)
{
    const short days_since_beginning_of_year[12] = {0,31,59,90,120,151,181,212,243,273,304,334};

    int leap_years = ((year-1)-1968)/4
                  - ((year-1)-1900)/100
                  + ((year-1)-1600)/400;

    long days_since_1970 = (year-1970)*365 + leap_years
                      + days_since_beginning_of_year[month-1] + day-1;

    if ( (month>2) && (year%4==0 && (year%100!=0 || year%400==0)) )
    days_since_1970 += 1; /* +leap day, if year is a leap year */

    return sec + 60 * ( min + 60 * (hour + 24*days_since_1970) );
}