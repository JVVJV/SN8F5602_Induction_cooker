/*--------------------------------------------------------------------------
SN8F5602.H

Header file for SNOiX SN8F5602 microcontroller.
Copyright (c) 2024 SONiX Technology Co., Ltd.

Version 1.0    2024-12-23
Version 1.1    2025-03-10 Update EPW0 error
--------------------------------------------------------------------------*/

#ifndef __SN8F5602_H__
#define __SN8F5602_H__

#define DEFINE_SFR(Register ,Address) \
				sfr Register = Address;

#ifdef __C51__  //** Compiler: C51
		#ifdef __XRAM_SFR_H__
			#define DEFINE_XSFR(Register ,Address) \
							volatile unsigned char xdata Register	_at_	Address;
		#else
			#define DEFINE_XSFR(Register ,Address) \
							extern volatile unsigned char xdata Register;
		#endif //for C51 __XRAM_SFR_H__ 
#else    //** Compiler: A51
		#ifdef __XRAM_SFR_H__
			#define DEFINE_XSFR(Register ,Address) \
							Register xdata Address; \
							PUBLIC Register
		#else
			#define DEFINE_XSFR(Register ,Address) \
							EXTRN xdata (Register)
		#endif //for C51 __XRAM_SFR_H__
#endif 

// --------------------------- //
//    8051 SFR Registers  		 //
// --------------------------- //
	DEFINE_SFR(P0			,0x80)
	DEFINE_SFR(SP 		,0x81)
	DEFINE_SFR(DPL 		,0x82)
	DEFINE_SFR(DPH 		,0x83)
	DEFINE_SFR(DPL1 	,0x84)
	DEFINE_SFR(DPH1 	,0x85)
	DEFINE_SFR(WDTR 	,0x86)
	DEFINE_SFR(PCON 	,0x87)
	DEFINE_SFR(TCON		,0x88)
	DEFINE_SFR(PEDGE	,0x8F)

	DEFINE_SFR(P1			,0x90)
	DEFINE_SFR(DPS		,0x92)
	DEFINE_SFR(DPC 		,0x93)
	DEFINE_SFR(SMODMX	,0x94)
	DEFINE_SFR(SMODCAL,0x95)
	DEFINE_SFR(S0CON2 ,0x96)
	DEFINE_SFR(S0CON 	,0x97)

	DEFINE_SFR(IEN3 	,0x98)
	DEFINE_SFR(S0BUF	,0x99)
	DEFINE_SFR(S0RELL	,0x9A)
	DEFINE_SFR(S0RELH	,0x9B)
	DEFINE_SFR(S1CON	,0x9C)
	DEFINE_SFR(S1BUF	,0x9D)
	DEFINE_SFR(S1RELL ,0x9E)
	DEFINE_SFR(S1RELH	,0x9F)

	DEFINE_SFR(IRCON2 ,0xA7)
	DEFINE_SFR(IEN0 	,0xA8)
	DEFINE_SFR(IP0		,0xA9)
	DEFINE_SFR(FRQL		,0xAA)
	DEFINE_SFR(FRQH		,0xAB)
	DEFINE_SFR(FRQCMD	,0xAC)
	DEFINE_SFR(LFRQL	,0xAD)
	DEFINE_SFR(LFRQH 	,0xAE)
	DEFINE_SFR(CLKCAL	,0xAF)

	DEFINE_SFR(PW0M		,0xB1)
	DEFINE_SFR(PW0YL 	,0xB2)
	DEFINE_SFR(PW0YH 	,0xB3)
	DEFINE_SFR(PW0DL 	,0xB4)
	DEFINE_SFR(PW0DH 	,0xB5)
	DEFINE_SFR(PW0M1 	,0xB6)	
	DEFINE_SFR(IEN1 	,0xB8)  
  DEFINE_SFR(IP1  	,0xB9) 
  DEFINE_SFR(PECMD 	,0xBA) 
  DEFINE_SFR(PEROML	,0xBB) 
  DEFINE_SFR(PEROMH	,0xBC) 
  DEFINE_SFR(PERAM 	,0xBD) 
  DEFINE_SFR(PERAMCNT 	,0xBE) 
  DEFINE_SFR(PEROMHH   	,0xBF) 

	DEFINE_SFR(P4    	,0xC0)
	DEFINE_SFR(DEGCMD ,0xC1)
  DEFINE_SFR(P1W    ,0xC3)
	DEFINE_SFR(IEN2	  ,0xC8)
	DEFINE_SFR(IRCON3	,0xC9)
	DEFINE_SFR(CMOUT	,0xCA)
	
	DEFINE_SFR(PSW		,0xD0)
	DEFINE_SFR(ADCAL	,0xD1)
	DEFINE_SFR(ADM 		,0xD2)
	DEFINE_SFR(ADB 		,0xD3)
	DEFINE_SFR(ADR	 	,0xD4)
	DEFINE_SFR(VREFH 	,0xD5)
	DEFINE_SFR(CLKSEL ,0xD6)
	DEFINE_SFR(CLKCMD	,0xD7)
	DEFINE_SFR(BZM 		,0xD9)
	DEFINE_SFR(I2CDAT	,0xDA)
	DEFINE_SFR(I2CADR	,0xDB)
	DEFINE_SFR(I2CCON	,0xDC)
	DEFINE_SFR(I2CSTA	,0xDD)
	DEFINE_SFR(SMBSEL	,0xDE)
	DEFINE_SFR(SMBDST	,0xDF)

	DEFINE_SFR(ACC		,0xE0)
  DEFINE_SFR(CKSUMCON 	,0xE4)
  DEFINE_SFR(CKSUMDATL 	,0xE5)
  DEFINE_SFR(CKSUMDATH 	,0xE6)
  DEFINE_SFR(LVDCON ,0xE7)
  
	DEFINE_SFR(B			,0xF0)
	DEFINE_SFR(CRCM 	,0xF1)
	DEFINE_SFR(CRCDATL,0xF2)
	DEFINE_SFR(CRCDATH,0xF3)
	DEFINE_SFR(SYSMOD ,0xF4)	
	DEFINE_SFR(SRST 	,0xF7)
	DEFINE_SFR(P0M		,0xF9)
	DEFINE_SFR(P1M 		,0xFA)
  DEFINE_SFR(P4M 		,0xFD)
	DEFINE_SFR(PFLAG	,0xFF)

// --------------------------- //
//    8051 16bit SFR 			 		 //
// --------------------------- //
	sfr16	S0REL		= 0x9A;
	sfr16	S1REL		= 0x9E;
	sfr16	FRQ			= 0xAA;
	sfr16	LFRQ		= 0xAD;	
	sfr16	PW0Y		= 0xB2;
	sfr16	PW0D		= 0xB4;	
  sfr16 PEROM   = 0xBB;  
  sfr16 CRCDAT  = 0xF2;
  
// --------------------------- //
//    8051 BIT SFR 	  		 		 //
// --------------------------- //
	/*	P0	*/
	sbit	P07	= P0^7;
	sbit	P06	= P0^6;
	sbit	P05	= P0^5;
	sbit	P04	= P0^4;
	sbit	P03	= P0^3;
	sbit	P02	= P0^2;
	sbit	P01	= P0^1;
	sbit	P00	= P0^0;
	/*	P1	*/
	sbit	P16	= P1^6;
	sbit	P15	= P1^5;
	sbit	P14	= P1^4;
	sbit	P13	= P1^3;
	sbit	P12	= P1^2;
	sbit	P11	= P1^1;
	sbit	P10	= P1^0;
  /*  P4  */
  sbit  P42 = P4^2;
  sbit  P41 = P4^1;
  sbit  P40 = P4^0;
 
	/*	TCON	*/
	sbit	TF1	= TCON^7;
	sbit	TF0	= TCON^5;
	sbit	IE1	= TCON^3;
	sbit	IE0	= TCON^1;
	/*	IEN0	*/
	sbit  EAL   = IEN0^7;
	sbit  ET1   = IEN0^3;
  sbit  ET0   = IEN0^1;
  sbit  EX0   = IEN0^0;
	/*	IEN1	*/
  sbit  EU1RX = IEN1^5;
  sbit  EU1TX = IEN1^4;
  sbit  EU0RX = IEN1^3;
  sbit  EU0TX = IEN1^2;
  sbit  EI2C  = IEN1^0;
	/*	IEN2	*/
  sbit  ECMP4	= IEN2^7;
  sbit  ECMP3 = IEN2^6;
  sbit  ECMP2	= IEN2^5;
  sbit  ECMP1 = IEN2^4;
  sbit  ECMP0 = IEN2^3;
	sbit  ET2 	= IEN2^2;
	sbit  EADC  = IEN2^1;
	sbit  ELVD  = IEN2^0;
	/*	IEN3	*/
  sbit  EPW0	= IEN3^2;	
	/*	PSW	*/
	sbit  CY	= PSW^7;
	sbit  AC	= PSW^6;
	sbit  F0	= PSW^5;
	sbit  RS1	= PSW^4;
	sbit  RS0	= PSW^3;
	sbit  OV	= PSW^2;
	sbit  F1	= PSW^1;
	sbit  P		= PSW^0;
// ------------------------------- //
//    8051 XRAM SFR Registers  		 //
// ------------------------------- //
	DEFINE_XSFR(P0UR		,0xF000)
	DEFINE_XSFR(P1UR		,0xF001)
	DEFINE_XSFR(P4UR		,0xF004)
	DEFINE_XSFR(P0CON 	,0xF010)
	DEFINE_XSFR(P1CON 	,0xF011)
  DEFINE_XSFR(P4CON 	,0xF014)
  DEFINE_XSFR(ODCON 	,0xF018)
	
	DEFINE_XSFR(OPM 		,0xF020)
	DEFINE_XSFR(OPGS		,0xF021)
	DEFINE_XSFR(OPCAL		,0xF022)
	DEFINE_XSFR(OPCAL1	,0xF023)	
  DEFINE_XSFR(CM0M	  ,0xF028)
  DEFINE_XSFR(CMDB0	  ,0xF029)
  DEFINE_XSFR(CM0CAL  ,0xF02A)
  DEFINE_XSFR(CM0CAL1 ,0xF02B)	
  DEFINE_XSFR(CM1M	  ,0xF02C)
  DEFINE_XSFR(CMDB1	  ,0xF02D)
  DEFINE_XSFR(CM1REF	,0xF02E)
  DEFINE_XSFR(CM2M	  ,0xF030)
  DEFINE_XSFR(CMDB2	  ,0xF031)
  DEFINE_XSFR(CM2REF  ,0xF032)
  DEFINE_XSFR(CM3M	  ,0xF034)
  DEFINE_XSFR(CMDB3	  ,0xF035)
  DEFINE_XSFR(CM3REF  ,0xF036)
  DEFINE_XSFR(CM4M	  ,0xF038)
  DEFINE_XSFR(CMDB4	  ,0xF039)
  DEFINE_XSFR(CM4REF  ,0xF03A)

  DEFINE_XSFR(INTREF  ,0xF040)

  DEFINE_XSFR(T0M 	  ,0xF060)
  DEFINE_XSFR(T0CH	  ,0xF061)
  DEFINE_XSFR(T0CL	  ,0xF062)
  DEFINE_XSFR(T0RH	  ,0xF063)
  DEFINE_XSFR(T0RL	  ,0xF064)
  DEFINE_XSFR(T1M 	  ,0xF068)
  DEFINE_XSFR(T1CH	  ,0xF069)
  DEFINE_XSFR(T1CL	  ,0xF06A)
  DEFINE_XSFR(T1RH	  ,0xF06B)
  DEFINE_XSFR(T1RL	  ,0xF06C)
  DEFINE_XSFR(T2M 	  ,0xF070)
  DEFINE_XSFR(T2CH	  ,0xF071)
  DEFINE_XSFR(T2CL	  ,0xF072)
  DEFINE_XSFR(T2RH	  ,0xF073)
  DEFINE_XSFR(T2RL	  ,0xF074)
  DEFINE_XSFR(P0PU    ,0xF0B0)
  DEFINE_XSFR(P1PU 	  ,0xF0B1)
  DEFINE_XSFR(P4PU    ,0xF0B4)	
	


/*      Interrupt Vector      */
#define ISRInt0    0     /* Vector 0x03 */
#define ISRTimer0  1     /* Vector 0x0B */
#define ISRTimer1  3     /* Vector 0x1B */
#define ISRTimer2  5     /* Vector 0x2B */
#define ISRI2c     8     /* Vector 0x43 */
#define ISRUart0rx 10    /* Vector 0x53 */
#define ISRUart0tx 11    /* Vector 0x5B */
#define ISRUart1rx 12    /* Vector 0x63 */
#define ISRUart1tx 13    /* Vector 0x6B */
#define ISRPwm0    16    /* Vector 0x83 */
#define ISRAdc     21    /* Vector 0xAB */
#define ISRLVD     23    /* Vector 0xBB */
#define ISRCmp0  	 24    /* Vector 0xC3 */
#define ISRCmp1		 25    /* Vector 0xCB */
#define ISRCmp2		 26    /* Vector 0xD3 */
#define ISRCmp3		 27    /* Vector 0xDB */
#define ISRCmp4    28    /* Vector 0xE3 */


/*      C51 Macros      */
#ifdef __C51__
	#include <intrins.h>
	#define IDLE() {PCON |= 0x01;}
	#define STOP() {PCON |= 0x02;}
	#define PISP(ROM_ADDRESS, RAM_ADDRESS) {PERAM = (RAM_ADDRESS); PEROM = ((ROM_ADDRESS) & 0xFFE0); PECMD = 0x5A; _nop_(); _nop_();}
	#define BISP(ROM_ADDRESS, RAM_ADDRESS) {PERAM = (RAM_ADDRESS); PEROM = ((ROM_ADDRESS) & 0xFFFF); PECMD = 0x1E; _nop_(); _nop_();}

#else // for A51
	WRXSFR	MACRO	XADDR, XDA
					MOV	  DPTR, #XADDR
					MOV	  A, XDA
					MOVX  @DPTR, A
					ENDM

	RDXSFR	MACRO	XDA, XADDR
					MOV	  DPTR, #XADDR
					MOVX  A, @DPTR
					MOV   XDA, A
					ENDM
#endif  

#ifndef uint8_t
#define uint8_t unsigned char
#endif  // uint8_t

#ifndef uint16_t
#define uint16_t unsigned int
#endif  // uint16_t

#ifndef uint32_t
#define uint32_t unsigned long
#endif  // uint32_t

#endif  // __SN8F5869_H__
