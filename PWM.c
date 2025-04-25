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
#include "PWM.h"
#include "config.h"
#include "power.h"
#include "system.h"
#include "comparator.h"

/*_____ D E C L A R A T I O N S ____________________________________________*/


/*_____ D E F I N I T I O N S ______________________________________________*/


/*_____ M A C R O S ________________________________________________________*/

/*_____ F U N C T I O N S __________________________________________________*/

void PWM_Init(void)
{
  PW0M = 0;
  PW0Y = PWM_MAX_WIDTH; // 設定 PW0Y 為最大值 patch, 防止不小心反向
  
  // Do not enable SFDL, as turning it on introduces a bug when reading/write PW0D.
  //PW0M1 = mskSFDL;
  
  PW0M = mskPW0EN | PW0_DIV1 | PW0_HOSC | PW0_INVERS;
}



/**
 * PWM0_ISR handles:
 * - Slowdown ramping during periodic heating
 * - High-voltage PW0D adjustment request from CMP2 (via PW0D_req_CMP2_isr )
 * - Frequency jitter logic for EMI reduction
 *
 * Handle CMP2 PW0D adjustment request.
 * This ensures PW0D is only written from one context (ISR or main) to avoid hardware glitch.
 * If processed here, clear the flag and exit early to avoid overlapping with jitter control.
 *
 */

//volatile uint16_t PW0D_val_ISR = 0;
//volatile uint16_t PW0D_val_main = 0;

void PWM0_ISR(void) interrupt ISRPwm0
{
  int tmp;
  
  if(PW0D_lock)
  {
    return;
  }
  
  // IGBT_C_slowdown ramp-up
  if ((system_state == PERIODIC_HEATING) &&
      (periodic_heat_state == PERIODIC_SLOWDOWN_PHASE)) 
  {
    if (PW0D < SLOWDOWN_PWM_MAX_WIDTH) {
        PW0D++;
    } else {
        PW0D = SLOWDOWN_PWM_MAX_WIDTH;
    }
    
    return;
  }

  // In heating logic
  if (f_heating_initialized) {
    // IGBT_High-voltage protection (decrease PW0D)
    if (PW0D_req_CMP2_isr) {
      PW0D_req_CMP2_isr = FALSE;

      if ((PW0D >= (3 + PWM_MIN_WIDTH))) {
        PW0D -= 3;
      } else {
        PW0D = PWM_MIN_WIDTH;
      }
      
      if (!f_jitter_active) 
      {PWM_INTERRUPT_DISABLE; }
      
      return;
    }
    
    // Power_Control request (±1)
    if (PW0D_delta_req_pwr_ctrl) {
      tmp = (int)PW0D + PW0D_delta_req_pwr_ctrl;
      
      if (tmp < PWM_MIN_WIDTH)      tmp = PWM_MIN_WIDTH;
      else if (tmp > PWM_MAX_WIDTH) tmp = PWM_MAX_WIDTH;
      
      PW0D = (uint16_t)tmp;
      
      PW0D_delta_req_pwr_ctrl = 0;
      
      if (!f_jitter_active) 
      {PWM_INTERRUPT_DISABLE; }
      
      return;
    }
    
    // Frequency jitter control
    if (Frequency_jitter_state == JITTER_DECREASE) {
      if (PW0D > PWM_MIN_WIDTH) {
          PW0D--;
      }
    } else if (Frequency_jitter_state == JITTER_INCREASE) {
      if (PW0D < PWM_MAX_WIDTH) {
          PW0D++;
      }
    }
    
  }
  
}


