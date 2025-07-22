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
#include "timer2.h"
#include "system.h"
#include "config.h"
/*_____ D E C L A R A T I O N S ____________________________________________*/

/*_____ D E F I N I T I O N S ______________________________________________*/

/*_____ M A C R O S ________________________________________________________*/

/*_____ F U N C T I O N S __________________________________________________*/


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Timer2_Init() {
  T2M = 0;
  T2M |= T2_DIV_1 | T2_CLK_FHOSC;
  
  T2RH = (T2SF_RELOAD>>8);
  T2RL = T2SF_RELOAD;
  
  T2CH = (T2SF_RELOAD>>8);
  T2CL = T2SF_RELOAD;
	
  // Clear TF2
  IRCON2 &= ~mskTF2;

  // Patch: In the T2_ISR, it is necessary to clear T2CH/L to 0x0000 
  // to ensure that the next PWM pulse start does not occur simultaneously 
  // with T2OV and trigger T2SF.
  ET2 = 1;  

  // Enable T2SF
  T2M |= mskT2SF; 

}


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Timer2_ISR() interrupt ISRTimer2
{
  // T2 Patch
  T2CH = 0x00;
  T2CL = 0x00;
}
