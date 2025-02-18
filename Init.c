/*_____ I N C L U D E S ____________________________________________________*/
#include <SN8F5602.h>
#include "TestDefine.H"
#include "Init.H"

/*_____ D E C L A R A T I O N S ____________________________________________*/

/*_____ D E F I N I T I O N S ______________________________________________*/

/*_____ M A C R O S ________________________________________________________*/

/*_____ F U N C T I O N S __________________________________________________*/
 
/*****************************************************************************
* Function		: 
* Description	: 
* Input			: None
* Output		: None
* Return		: None
* Note			: None
*****************************************************************************/
void Sys_init(void)
{
  //** Flosc clockcontrol by Code Option
  //** Fcpu clock control by Code Option
  DEGCMD = 0x00;              //** 0xA5 = Enable OCDS module, ohters = Disable OCDS
  
  P0 = 0x00;
  P0M = 0xFF;
  
  EAL = 1;
  
  #if(STWK_EN==0)
  {
    SYSMOD = 0x00;    // The low clock source (Flosc) keeps stop in STOP mode.
  }
  #elif(STWK_EN==1)
  {
    SYSMOD = 0x01;    // The low clock source (Flosc) keeps running in STOP mode.
  }
  #endif
}

/*****************************************************************************
* Function		: 
* Description	: 
* Input			: None
* Output		: None
* Return		: None
* Note			: None
*****************************************************************************/
void TestIO_set(void)
{  
  TestPinData = 0x00;
  TestPinDir = 0xFF;
}


/*****************************************************************************
* Function		: 
* Description	:  
* Input			: None
* Output		: None
* Return		: None
* Note			: None
*****************************************************************************/
void WakeIO_set(void)
{  
  P0M &= ~(0x01<<P0WAKE_PININD);
}

/*****************************************************************************
* Function		: 
* Description	:  
* Input			: None
* Output		: None
* Return		: None
* Note			: None
*****************************************************************************/
void T0_init(void)
{
//	//** Timer0 initial setting
//	TH0 = 0x00;
//	TL0 = 0x00;
//  TCON0 |= (0x80 | 0x70) ;	//** T0 = Flosc/1 for 16ms timeout
//  
//	TMOD |= 0x06;		          //** T0 is 8-bit autoreload, clock=Fhosc/N
//	IEN0 |= 0x02; 	          //** T0 Interrupt
//  
//  TCON &= ~0x20;
//  TCON |= 0x10;             //** enable timer0 counter;
}

/*****************************************************************************
* Function		: 
* Description	:  
* Input			: None
* Output		: None
* Return		: None
* Note			: None
*****************************************************************************/
void T1_init(void)
{
//	//** Timer1 initial setting
//	TH1 = 0x00;
//	TL1 = 0x00;
//	//TCON0 |= 0x00;	// T1 = Fhosc/128
//	//TCON0 |= 0x01;	// T1 = Fhosc/64
//  TCON0 |= (0x80 | 0x07) ;	//** T1 = Flosc/1 @ILRC for 500us timeout
//  
//	TMOD |= 0x60;		// T1 is 8-bit autoreload, clock=Fhosc/N
//	IEN0 |= 0x08; 	// T1 Interrupt
//  
//  TCON |= 0x40;   //** enable timer1 counter;
}

/*****************************************************************************
* Function		: 
* Description	:  
* Input			: None
* Output		: None
* Return		: None
* Note			: None
*****************************************************************************/
void T3_init(void)
{
//	//** Timer3 initial setting
//	T3CH = 0x5A;
//	T3CL = 0xA5;
//  T3RH = 0x00;
//  T3RL = 0x00;

//  T3M |= 0x80 | 0x08;  //** enable timer3 counter, clock=Fhosc;

//	IEN2 |= 0x20; 	     //** T3 Interrupt
   
}

/*****************************************************************************
* Function		: 
* Description	:  
* Input			: None
* Output		: None
* Return		: None
* Note			: None
*****************************************************************************/
void TT_init(void)
{
//	//** TT initial setting
//	TTH = 0x00;
//	TTL = 0x00;
//	TTCON |=  (0x20 | 0x08) ;
//  TTCON |=  0x07;

//	TTCON |=  0x80; //** enable TT counter
//  
//  
//  IEN3 |= 0x02;   //** TT Interrupt
}

/*****************************************************************************
* Function		: 
* Description	:  
* Input			: None
* Output		: None
* Return		: None
* Note			: None
*****************************************************************************/
void ADC_init(void)
{
	//** Normal ADCIP enable
	    
  ADM = 0x80 | 0x09;          //** bit7 = IP enable bit, bit4~bit0 = channel select
                              //** enable, CHS = AIN9
  ADR = 0x80 | 0x40 | 0x20;   //** bit7 = clock source = Fhosc/Flosc, bit6 = global channel enable, bit5,4 = clock select
                              //** clock = Fhosc/32 => 64us converting time
  VREFH = 0x7F & 0x02;
  
  ADM |= 0x40;
  
  IEN2 |= 0x01;
}

/*****************************************************************************
* Function		: 
* Description	:  
* Input			: None
* Output		: None
* Return		: None
* Note			: None
*****************************************************************************/
void delay_func(void)
{
  unsigned int i;
  for(i=0;i<0x3FFF;i++);
  
}