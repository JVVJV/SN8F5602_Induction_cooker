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
  //CMDB0 = DELAY_0T | DEBOUNCE_0FCPU;   //HCW** for experiment
  
  CM0M |= mskCM0EN;
  
  //CM0_IRQ_ENABLE; //HCW** for experiment
  
  // INTERNAL REFERENCE VOLTAGE -------------------------------------------------------
  INTREF = mskINTREFEN | INTREF3V5;
  
  // CM1 -------------------------------------------------------
  // AC Power Sync.
  CM1M = 0;
  // CM1N
  // P04 switch to analog pin
  P0CON |= (1<<4);  
  // CM1P
  // 設定 CM1+ 端為內部參考電壓 0.5V
  // INTERNAL 3.5V*9/64 = 0.5V (0.49V) -> CM1RS = 9 
  CM1REF = CM1REF_INTREF | 9;
  
  CM1M = mskCM1EN | CM1_RISING_TRIGGER;
  
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


#define AC_SYNC_DEBOUNCE_TICKS 32  // **AC 週期同步去抖動時間 32*125us (4ms)**
/*
   AC_SYNC_DEBOUNCE_TICKS 用於確保 `f_CM1_AC_sync` 只會在新的 AC 週期開始時觸發：
   1. 當 `CM1-` 低於 `CM1+` (0.5V) 時，`CM1_ISR` 會觸發，但可能會有訊號抖動。
   2. 此變數設定 `4ms (32 system_ticks)` 作為 debounce 時間。
   3. `f_CM1_AC_sync` 只在 `AC_SYNC_DEBOUNCE_TICKS` 之後才允許再次觸發，防止抖動影響計時準確性。
*/

volatile bit f_CM1_AC_sync = 0;
volatile uint8_t last_sync_tick = 0; // 記錄上次 `f_CM1_AC_sync` 設為 `1` 的時間

void comparator1_ISR(void) interrupt ISRCmp1
{
  // **確保 `f_CM1_AC_sync` 只會在新的 AC 週期內觸發**
  if ((uint8_t)(system_ticks - last_sync_tick) >= AC_SYNC_DEBOUNCE_TICKS) {
    f_CM1_AC_sync = 1;       // **標記新 AC 週期開始**
    last_sync_tick = system_ticks; // **更新 `last_sync_tick`**
  }
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
