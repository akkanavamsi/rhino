
#include "ad5940main.h"

#include "definitions.h" 
#include "BodyImpedance.h"
//#include "mpfit.h"
#include "Calibration/figaro_gpr_fit.h"
#include <math.h>
#include "console.h"
#include "../src/auxillary_sensors.h"

#define APPBUFF_SIZE 512

AD5940_DATA ad5940Data;
uint32_t AppBuff[APPBUFF_SIZE];
uint32_t IntCount;
uint32_t count;
uint32_t temp;

double figaroPPM;

figaro_result_t figaroResult;

int32_t BIAShowResult(uint32_t *pData, uint32_t DataCount);
void printConsole(float f);

/* Initialize AD5940 basic blocks like clock */
int32_t AD5940PlatformCfg(void)
{
  CLKCfg_Type clk_cfg;
  FIFOCfg_Type fifo_cfg;
  AGPIOCfg_Type gpio_cfg;

  /* Use hardware reset */
  AD5940_HWReset();
  /* Platform configuration */
  AD5940_Initialize();
  /* Step1. Configure clock */
  clk_cfg.ADCClkDiv = ADCCLKDIV_1;
  clk_cfg.ADCCLkSrc = ADCCLKSRC_HFOSC;
  clk_cfg.SysClkDiv = SYSCLKDIV_1;
  clk_cfg.SysClkSrc = SYSCLKSRC_HFOSC;
  clk_cfg.HfOSC32MHzMode = bFALSE;
  clk_cfg.HFOSCEn = bTRUE;
  clk_cfg.HFXTALEn = bFALSE;
  clk_cfg.LFOSCEn = bTRUE;
  AD5940_CLKCfg(&clk_cfg);
  /* Step2. Configure FIFO and Sequencer*/
  fifo_cfg.FIFOEn = bFALSE;
  fifo_cfg.FIFOMode = FIFOMODE_FIFO;
  fifo_cfg.FIFOSize = FIFOSIZE_4KB;                       /* 4kB for FIFO, The reset 2kB for sequencer */
  fifo_cfg.FIFOSrc = FIFOSRC_DFT;
  fifo_cfg.FIFOThresh = 4;//AppBIACfg.FifoThresh;        /* DFT result. One pair for RCAL, another for Rz. One DFT result have real part and imaginary part */
  AD5940_FIFOCfg(&fifo_cfg);                             /* Disable to reset FIFO. */
  fifo_cfg.FIFOEn = bTRUE;  
  AD5940_FIFOCfg(&fifo_cfg);                             /* Enable FIFO here */
  
  /* Step3. Interrupt controller */
  
  AD5940_INTCCfg(AFEINTC_1, AFEINTSRC_ALLINT, bTRUE);           /* Enable all interrupt in Interrupt Controller 1, so we can check INTC flags */
  AD5940_INTCCfg(AFEINTC_0, AFEINTSRC_DATAFIFOTHRESH, bTRUE);   /* Interrupt Controller 0 will control GP0 to generate interrupt to MCU */
  AD5940_INTCClrFlag(AFEINTSRC_ALLINT);
  /* Step4: Reconfigure GPIO */
  gpio_cfg.FuncSet = GP6_SYNC|GP5_SYNC|GP4_SYNC|GP2_TRIG|GP1_SYNC|GP0_INT;
  gpio_cfg.InputEnSet = AGPIO_Pin2;
  gpio_cfg.OutputEnSet = AGPIO_Pin0|AGPIO_Pin1|AGPIO_Pin4|AGPIO_Pin5|AGPIO_Pin6;
  gpio_cfg.OutVal = 0;
  gpio_cfg.PullEnSet = 0;

  AD5940_AGPIOCfg(&gpio_cfg);
  AD5940_SleepKeyCtrlS(SLPKEY_UNLOCK);  /* Allow AFE to enter sleep mode. */
  return 0;
}

void AD5940MAIN_Initialize(void)
{
    uint32_t x;
    for(x = 0; x < 1.5E6; x++)
    {
        asm("NOP");
        asm("NOP");
        asm("NOP");
    }
    ad5940Data.state = AD5940_STATE_INIT;
}

uint32_t testID;
uint32_t testCHIP;

void AD5940BIAStructInit(void)
{
  AppBIACfg_Type *pBIACfg;
  
  AppBIAGetCfg(&pBIACfg);
  
  pBIACfg->SeqStartAddr = 0;
  pBIACfg->MaxSeqLen = 512; /** @todo add checker in function */
  
  pBIACfg->RcalVal = 10000.0;
  pBIACfg->DftNum = DFTNUM_8192;
  pBIACfg->NumOfData = -1;      /* Never stop until you stop it manually by AppBIACtrl() function */
  pBIACfg->BiaODR = 24.0;         /* ODR(Sample Rate) 20Hz */
  pBIACfg->FifoThresh = 4;      /* 4 */
  pBIACfg->ADCSinc3Osr = ADCSINC3OSR_2;
}

uint32_t x;

void AD5940MAIN_Tasks(void)
{
    switch(ad5940Data.state)
    {
        case AD5940_STATE_INIT:
            SPI5CONbits.ON = 1;
            AD5940_MCUResourceInit(0);
            x = AD5940PlatformCfg();
            AD5940BIAStructInit();
            x =  AppBIAInit(AppBuff, APPBUFF_SIZE);    /* Initialize BIA application. Provide a buffer, which is used to store sequencer commands */
            x = AppBIACtrl(BIACTRL_START, 0);         /* Control BIA measurement to start. Second parameter has no meaning with this command. */
            ad5940Data.state = AD5940_STATE_RUN;
            break;
        case AD5940_STATE_RUN:
            /* Check if interrupt flag which will be set when interrupt occurred. */
            if(AD5940_GetMCUIntFlag())
            {
              IntCount++;
              AD5940_ClrMCUIntFlag(); /* Clear this flag */
              temp = APPBUFF_SIZE;
              AppBIAISR(AppBuff, &temp); /* Deal with it and provide a buffer to store data we got */
              //testFig();
              BIAShowResult(AppBuff, temp); /* Show the results to UART */

              if(IntCount == 240)
              {
                IntCount = 0;
                //AppBIACtrl(BIACTRL_SHUTDOWN, 0);
              }
            }
            count++;
            break;
            case AD5940_STATE_ERROR:
            break;
        default:
            break;
    }
}
  double figReal[25];
  double figImag[25];
/* It's your choice here how to do with the data. Here is just an example to print them to UART */
int32_t BIAShowResult(uint32_t *pData, uint32_t DataCount)
{
  float freq;
  char str[80];
	
	// condition degueu marker
  //**** Test *****
   
  
  double RH = 46.0;
  double T = 25.0;
  double P;
  
  getAuxSensorAveragesD(&RH, &T, &P);
  
  double R;
  double C;

  fImpPol_Type *pImp = (fImpPol_Type*)pData;

  double mag;
  double angle;

  static int index = 0;

  // Get the frea signalled to find round about 
  AppBIACtrl(BIACTRL_GETFREQ, &freq);
  if(((int) freq) == 2000) 
  {
    index = 0;
    figaroResult = figaro_r_t_rh_calibration_realimag_error(figReal, figImag, T, RH, &figaroPPM, &R, &C);
    if(figaroPPM < 0)
        figaroPPM = 0;
    if(figaroPPM != figaroPPM)//NAN
        figaroPPM = 0;

//    sprintf(str, "");
//    sprintf(str, "Figaro PPM: %f\n", figaroPPM);
//    printConsoleStr(str);
  }

  mag = pImp[0].Magnitude;
  angle = pImp[0].Phase;

  figReal[index] = mag * cos(angle);
  figImag[index] = mag * sin(angle);
  
//  sprintf(str, "");
//  sprintf(str, "Freq: %f - Real: %f - Imag: %f\n", freq, figReal[index], figImag[index]);
//  printConsoleStr(str);

  if(index++ > 25) {
      sprintf(str, "");
      sprintf(str, "Overrun\n");
      printConsoleStr(str);
      index = 0;
  }
  return 0;
}