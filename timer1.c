/******************** (C) COPYRIGHT 2024 SONiX *******************************
* COMPANY:	SONiX
* DATE:		  2025/07
* AUTHOR:		HCW
* IC:			  SR56F27
*____________________________________________________________________________
* REVISION		Date				User		Description
*____________________________________________________________________________
*****************************************************************************/

/*_____ I N C L U D E S ____________________________________________________*/
#include "timer1.h"
#include "system.h"
#include "config.h"

/*_____ D E C L A R A T I O N S ____________________________________________*/

/*_____ D E F I N I T I O N S ______________________________________________*/

/*_____ M A C R O S ________________________________________________________*/

/*_____ F U N C T I O N S __________________________________________________*/


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#define T1SF_RELOAD     (uint16_t)(0xFFFF-(1000*32)+1) // 1000us @32MHz

void Timer1_Init() {
  T1M = 0;
  T1M |= T1_DIV_1 | T1_CLK_FHOSC | mskT1SF;
  
  T1RH = (T1SF_RELOAD>>8);
  T1RL = T1SF_RELOAD;
  
  T1CH = (T1SF_RELOAD>>8);
  T1CL = T1SF_RELOAD;
	
  // Clear TF1
  TCON &= ~mskTF1;

  ET1 = 1;
}

void Timer1_Enable(void) {
  TCON &= ~mskTF1;    // Clear TF1
  T1M |= mskT1EN;
  
}

void Timer1_Disable(void) {
  T1CH = (T1SF_RELOAD>>8);
  T1CL = T1SF_RELOAD;
  T1M &= ~mskT1EN;
  
}


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Timer1_ISR() interrupt ISRTimer1
{
  // T1SF indicates unexpected halt
  ISR_f_Unexpected_halt = 1;
  
  // Timer1 disable
  T1M &= ~mskT1EN;
  
}
