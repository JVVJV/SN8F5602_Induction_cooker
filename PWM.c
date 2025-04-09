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

/*_____ D E C L A R A T I O N S ____________________________________________*/


/*_____ D E F I N I T I O N S ______________________________________________*/


/*_____ M A C R O S ________________________________________________________*/

/*_____ F U N C T I O N S __________________________________________________*/

void PWM_Init(void)
{
  PW0M = 0;
  PW0Y = PWM_MAX_WIDTH; // 設定 PW0Y 為最大值 patch, 防止不小心反向
  //PW0D = 0x100;       // 設定 PW0D 為最大值 patch
  
  PW0M1 = mskSFDL;
  
  PW0M = mskPW0EN | PW0_DIV1 | PW0_HOSC | PW0_INVERS;
  
}

void PWM0_ISR(void) interrupt ISRPwm0
{
    if ((system_state == PERIODIC_HEATING) &&
        (periodic_heat_state == PERIODIC_SLOWDOWN_PHASE)) 
    {
        if (PW0D < SLOWDOWN_PWM_MAX_WIDTH) {
            PW0D++;
        } else {
            PW0D = SLOWDOWN_PWM_MAX_WIDTH;
        }
    }

}


