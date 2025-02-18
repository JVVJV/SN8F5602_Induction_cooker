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
  // CM0
  CM0M = 0;
  // P02, P03 switch to analog pin
  P0CON |= (1<<3)|(1<<2); 
  
  CM0M = CM0_FALLING_TRIGGER | CM0_CLK_FCPU;
  CMDB0 = DELAY_4T | DEBOUNCE_10FCPU;
  CM0M |= mskCM0EN;
  
  // CM1
  
  
  
  // CM2 IGBT overvoltage protect @1100V (AC power High or Pot be taken off)
  CM2M = 0;
  // CM2N
  // P05 switch to analog pin
  P0CON |= (1<<5);  
  // CM2P
  // INTERNAL REFERENCE VOLTAGE
  INTREF = mskINTREFEN | INTREF3V5;
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
