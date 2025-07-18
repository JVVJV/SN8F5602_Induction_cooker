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
#include "timer0.h"
#include "system.h"

/*_____ D E C L A R A T I O N S ____________________________________________*/


/*_____ D E F I N I T I O N S ______________________________________________*/


/*_____ M A C R O S ________________________________________________________*/

/*_____ F U N C T I O N S __________________________________________________*/



//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#define TIMER0_RELOAD_VALUE (0xFFFF - 3999) //4000 * 32MHz = 125us

void Timer0_Init() {
  // 計算 Timer0 的初始值對應 32 MHz 系統時鐘
  T0M = 0;
  T0M |= DIV_1 | CLK_FHOSC;
  
  T0RH = (TIMER0_RELOAD_VALUE>>8);
  T0RL = TIMER0_RELOAD_VALUE;
  
  T0CH = (TIMER0_RELOAD_VALUE>>8);
  T0CL = TIMER0_RELOAD_VALUE;
    
  // clear TF0
  TCON &= ~mskTF0;
  // 啟用 Timer0 中斷
  ET0 = 1;

  // 啟用 Timer0 
  T0M |= mskT0EN;
  
}


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Timer0 中斷服務程式
void Timer0_ISR() interrupt ISRTimer0
{
    system_ticks++;       // 增加 125 μs 計數
    ISR_f_125us = 1;      // 立起 125 μs 旗標
  
}
