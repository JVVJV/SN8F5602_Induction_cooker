/******************** (C) COPYRIGHT 2024 SONiX *******************************
* COMPANY:	SONiX
* DATE:		  2025/01
* AUTHOR:		HCW
* IC:			  SR56F27
*____________________________________________________________________________
* REVISION		Date				User		Description
*____________________________________________________________________________
*****************************************************************************/

/*_____ I N C L U D E S ____________________________________________________*/
#include "comparator.h"
#include "config.h"
#include "system.h"
#include "power.h"

/*_____ D E C L A R A T I O N S ____________________________________________*/


/*_____ D E F I N I T I O N S ______________________________________________*/


/*_____ M A C R O S ________________________________________________________*/

/*_____ F U N C T I O N S __________________________________________________*/

void Comparator_Init(void)
{
  // CM0 -------------------------------------------------------
  CM0M = 0;
  // P02, P03 switch to analog pin
  P0CON |= (1<<3)|(1<<2); 
  
  CM0M = CM0_FALLING_TRIGGER | CM0_CLK_FCPU;
  CMDB0 = DELAY_4T | DEBOUNCE_10FCPU;
  
  CM0M |= mskCM0EN;
  
  //CM0_IRQ_ENABLE; //HCW** for experiment
  
  // INTERNAL REFERENCE VOLTAGE -------------------------------------------------------
  INTREF = mskINTREFEN | INTREF3V5;
  
  // CM1 -------------------------------------------------------
  // AC Power surge protection @260 Vrms
  // Protection voltage is 260V RMS (peak 367V), 
  // with a resistor divider ratio of 0.0049, resulting in 1.8V.
  
  // After the induction cooker begins operation, the GND potential shifts upward & down, so we need an increase in the voltage surge detection threshold.
  // For this prototype circuit, an upward adjustment of 225mV is required
  CM1M = 0;
  
  // CM1N
  // P04 switch to analog pin
  P0CON |= (1<<4); 
  
  // CM1P
  // 設定 CM1+ 端為內部參考電壓 1.8V + 0.225V = 2.025V
  // INTERNAL 3.5V*37/64 = 2.023V-> CM1RS = 37
  CM1REF = CM1REF_INTREF | 37;
  
  CMDB1 = CM1_DEBOUNCE_8FCPU; // 0.5us
  
  CM1M = mskCM1EN | mskCM1SF |CM1_FALLING_TRIGGER;
  
  IEN2 |= mskECMP1;
  
  // CM2 -------------------------------------------------------
  // IGBT overvoltage protect @1100V (AC power High or Pot be taken off)
  CM2M = 0;
  // CM2N
  // P05 switch to analog pin
  P0CON |= (1<<5);  
  
  // CM2P
  // 1100V*2.5K/827.5K = 3.32V
  // INTERNAL 3.5V*60/64 = 3.28V -> CM2RS = 60 
  CM2REF = CM2REF_INTREF | 60; 
  
  //CM2REF = CM2REF_INTREF | 38;  //700V
  
  CM2M = mskCM2EN | mskCM2SF |CM2_FALLING_TRIGGER;
  
  IEN2 |= mskECMP2;
  
  // CM3 -------------------------------------------------------
  // AC Power Sync.
  CM3M = 0;
  
  // CM3P
  // P04 switch to analog pin
  // P0CON |= (1<<4);    // It has already been executed during CM2 initialization.
  
  // CM3N
  // Set CM3- to internal reference voltage 0.5V
  // INTERNAL 3.5V*9/64 = 0.5V (0.49V) -> CM3RS = 9 
  CM3REF = CM3REF_INTREF | 9;
  
  //CM3REF = CM3REF_INTREF | 1; // 54mV
  
  CMDB3 = CM3_DEBOUNCE_20FCPU;
  
  CM3M = mskCM3EN | CM3_FALLING_TRIGGER;
  
  IEN2 |= mskECMP3;
  
}

void comparator0_ISR(void) interrupt ISRCmp0
{
  //P10 = 1; //HCW**
  
  if(pot_pulse_cnt < 250)
  {
    pot_pulse_cnt++; // 每次中斷觸發，脈衝計數器自增
  }    
  
  //P10 = 0; //HCW**
 
    // 其他 Comparator0 的邏輯可在此處添加
}

void comparator1_ISR(void) interrupt ISRCmp1
{  
  // 當 CM1 觸發中斷，代表電壓浪湧發生，立起 Surge_Overvoltage_Flag
  Surge_Overvoltage_Flag  = 1;
  
  // 中斷內 初步停止加熱邏輯
  P01 = 1;  //PWM Pin
  PW0M = 0;
}

void comparator2_ISR(void) interrupt ISRCmp2
{
  //P10 = 1; //HCW**
  
  // Patch for CM2SF can't be triggered by edge.
  if ((PW0D >= (3 + PWM_MIN_WIDTH))) { // 確保不小於最小值
      PW0D -= 1;
  } else {
      PW0D = PWM_MIN_WIDTH; // 避免低於最小寬度
  }
  
  //P10 = 0; //HCW**
}

#define AC_SYNC_DEBOUNCE_TICKS 32  // **AC 週期同步去抖動時間 32*125us (4ms)**
/*
   AC_SYNC_DEBOUNCE_TICKS is used to ensure that `f_CM3_AC_sync` is only triggered at the start of a new AC cycle:
   1. When `CM3+` is lower than `CM3-` (0.5V), `CM3_ISR` will be triggered, but there may be signal jitter.
   2. This variable sets `4ms (32 system_ticks)` as the debounce time.
   3. `f_CM3_AC_sync` is only allowed to be triggered again after `AC_SYNC_DEBOUNCE_TICKS`, preventing jitter from affecting the timing accuracy.
*/

volatile bit f_CM3_AC_sync = 0;
volatile uint8_t idata CM3_AC_sync_cnt = 0;
volatile uint8_t idata CM3_last_sync_tick = 0; // Record the time when `f_CM3_AC_sync` was set to `1`

void comparator3_ISR(void) interrupt ISRCmp3
{
  // **Ensure that `f_CM3_AC_sync` is only triggered within a new AC cycle**
  if ((uint8_t)(system_ticks - CM3_last_sync_tick) >= AC_SYNC_DEBOUNCE_TICKS) {
    f_CM3_AC_sync = 1;       // **Mark the start of a new AC cycle**
    CM3_AC_sync_cnt++;
    CM3_last_sync_tick = system_ticks; // **Update `last_sync_tick`**
  }
}