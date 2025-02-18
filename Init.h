/******************** (C) COPYRIGHT 2021 SONiX *******************************
* COMPANY:	SONiX
* DATE:		  2021/02
* AUTHOR:		Bochen
* IC:			  SH56E40
*____________________________________________________________________________
* REVISION		Date				User		Description
*____________________________________________________________________________
*****************************************************************************/
#ifndef _INIT_H_
#define _INIT_H_


/*_____ I N C L U D E S ____________________________________________________*/

/*_____ D E F I N I T I O N S ______________________________________________*/

/*_____ F U N C T I O N S __________________________________________________*/
extern void Sys_init(void);
extern void TestIO_set(void);
extern void WakeIO_set(void);
extern void delay_func(void);

extern void T0_init(void);
extern void T1_init(void);
extern void T3_init(void);
extern void TT_init(void);
extern void ADC_init(void);

#endif