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
  PW0Y = PWM_MAX_WIDTH; // �]�w PW0Y ���̤j�� patch, ����p�ߤϦV
  
  // HCW*** Do not enable SFDL, as turning it on introduces a bug when reading/write PW0D.
  //PW0M1 = mskSFDL;
  
  PW0M = mskPW0EN | PW0_DIV1 | PW0_HOSC | PW0_INVERS;
  
  // PWM0 Ineterrupt Priority, Group0 -> LEVEL3
  IP0 = (0<<5) | (0<<4) | (0<<3) | (0<<2) | (0<<1) | (1<<0);  
  IP1 = (0<<5) | (0<<4) | (0<<3) | (0<<2) | (0<<1) | (1<<0);  
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
    
    goto ISR_EXIT;
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
      
      goto ISR_EXIT;
    }
    
    // Power_Control request (��1)
    if (PW0D_delta_req_pwr_ctrl) {
      tmp = (int)PW0D + PW0D_delta_req_pwr_ctrl;
      
      if (tmp < PWM_MIN_WIDTH)      tmp = PWM_MIN_WIDTH;
      else if (tmp > PWM_MAX_WIDTH) tmp = PWM_MAX_WIDTH;
      
      PW0D = (uint16_t)tmp;
      
      PW0D_delta_req_pwr_ctrl = 0;
      
      if (!f_jitter_active) 
      {PWM_INTERRUPT_DISABLE; }
      
      goto ISR_EXIT;
    }
		
		if (PW0D_req_quick_surge) {
      P10 = ~P10;
			if ((PW0D >= (quick_surge_pwm_drop  + PWM_MIN_WIDTH))) {
        PW0D -= quick_surge_pwm_drop ;  // �� CMP2 ��֧�j�T��
    } else {
        PW0D = PWM_MIN_WIDTH;
    }

		PW0D_req_quick_surge = 0;
		
    if (!f_jitter_active) {
        PWM_INTERRUPT_DISABLE;
    }

			return;
		}
		
    
    // Frequency jitter control
    if (Frequency_jitter_state == JITTER_DECREASE) {
      if (PW0D > PWM_MIN_WIDTH) {
          PW0D--;
          jitter_adjust_cnt++;
      }
    } else if (Frequency_jitter_state == JITTER_INCREASE) {
      if ((PW0D < PWM_MAX_WIDTH) && (jitter_adjust_cnt > 0)) {
          PW0D++;
          jitter_adjust_cnt--;
        }
    }
  }// heating logic end
  
  ISR_EXIT:
  IRCON3 &= ~(1<<2); //PW0F_CLEAR
  
}// PWM_ISR end


