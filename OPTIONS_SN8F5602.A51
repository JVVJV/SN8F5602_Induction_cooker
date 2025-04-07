$NOMOD51
;------------------------------------------------------------------------------
;
;  Copyright (c) 2024 SONiX Technology Co., Ltd.
;  Version 1.0 - SN8F5602, SN8F5601, SN8F56011. Add code option address for MP5
;  *** <<< Use Configuration Wizard in Context Menu >>> ***
;------------------------------------------------------------------------------
;
;  This preference, such as watchdog, external reset pin, and clock source, is preloaded 
;  during the microcontroller's power-on. It is strongly recommanded to use configuration 
;  wizard to set these parameters up appropriately.
;
;------------------------------------------------------------------------------
ROM_SIZE		EQU		0x4800

;------------------------------------------------------------------------------
;	code option data setting
;------------------------------------------------------------------------------	
;   <h> LVD Reset Setting
;		<o.0..1> LVD14 & LVD18 de-bounce <0x03=> Disable <0x02=> 4*ILRC <0x01=> 8*ILRC <0x00=> 16*ILRC 
		LVD_DEBOUNCE_SET    EQU     0x3			;	{0x47FA}
;   <i> Disable: LVD no debounce.	
;   <i> 4*ILRC: LVD debounce 4*ILRC 
;   <i> 8*ILRC: LVD debounce 8*ILRC 
;   <i> 16*ILRC: LVD debounce 16*ILRC 

;		<o.6> LVD_IO reset function <0x1=> Enable <0x0=> Disable
		LVDESTEN_SET			    EQU     0x00			;	{0x47FA}
;   <i> Disable: Flag	
;   <i> Enable: Reset

;		<o.7> LVD_IO IP enable <0x1=> Enable <0x0=> Disable
		LVDEN_SET					    EQU     0x00			;	{0x47FA}
;   <i> LVD_IO IP enable.
;   </h>

;   <h> Clock Source Setting
;   <o.1>	Noise Detect2	<0x1=> Disable <0x0=>Enable
		HOLD_CK2_SET   				EQU     0x02		;	{0x47FB}
;   <i> Detect IO domain.

;   <o.5..7> CPU Clock Rate  <0x00=> Fhosc/1 <0x01=> Fhosc/2 <0x02=> Fhosc/4 <0x03=> Fhosc/8 <0x04=> Fhosc/16 <0x05=> Fhosc/32  <0x06=> Fhosc/64 <0x07=> Fhosc/128 
    CLOCKRATE_SET  			  EQU     0x20		;	{0x47FB}
;   <i> FCPU is Fhosc/1~Fhosc/128.
;   </h>

;   <h> Reset Sources
;   <o.3> Noise Detect1 <0x01=> 2ms <0x00=> 4ms
    HOLD_CK_SET    			EQU     0x08		;	{0x47FC}			
;   <i> Detect CPU domain.			
;   <o.4..5> External Reset / GPIO Shared Pin <0x00=> Reset with De-bounce <0x02=> Reset without De-bounce <0x03=> GPIO
		RESETPIN_SET   		  EQU     0x30		;	{0x15FC}
;   <i> Reset with De-bounce: Triggers reset if this pin is pulled low over 8 ms.
;   <i> Reset without De-bounce: Triggers reset immediately if this pin is pulled low.
;   <i> GPIO: The shared pin is reserved for general purpose input/output.
;   <i> The de-bounce period is based on Internal Low R-C Clock which has a gentle inaccuracy.
;   <o.6..7> Watchdog Overflow Period <0x00=> 64 ms <0x01=> 128 ms <0x02=> 256 ms <0x03=> 512 ms
    WATCHCLK_SET    		EQU     0x00		;	{0x15FC}
;   <i> The watchdog overflow period is based on Internal Low R-C Clock which has a gentle inaccuracy.

;  	<o.4..7> Watchdog Reset <0x00=> Disable <0x05=> Enable <0x0A=> Always On 
    WATCHDOG_SET   		  EQU     0xA0		;	{0x47FF}
;   <i> Disable: Turn off watchdog function.
;   <i> Enable: Turn on watchdog function only in Normal mode.
;   <i> Always: Trun on watchdog function including Normal, IDLE, and STOP mode.
;   </h>

;	<o.0..1> Program Memory Security <0x03=> Security Disable <0x02=> Security Enable.<0x00=> Security Configuration
	SECURITY_SET    EQU     0x03		;	{0x47FF}
;   <i> Security Disable: all address ROM data can be accessed.	
;   <i> Security Enable: all address ROM data are protected.
;   <i> Security Configuration: all address ROM data are protected expect address 0x4680 ~ 0x477F ROM data can be accessed.

;   <o.2> ISP Program Area  <0x01=> All Page <0x00=> Page 141~ Page 142
    ISP_EN_SET    EQU     0x04		;	{0x47FF}
;   <i> All Page: all address can perform ISP function.
;   <i> Page 141 ~ Page 142: only address 0x4680 ~ 0x477F can perform ISP function.
;	<o.3> CK_Fine_Tuning <0x01=> Disable <0x00=> Enable
    CK_FINE_TUNING_SET  EQU     0x08		;	{0x47FF}
;   <i> IHRC frequency fine tuning function.

;------------------------------------------------------------------------------
;	Code Option Data Mapping
;------------------------------------------------------------------------------
	CSEG	AT		 0x47FA
	DB      0x3C + LVDEN_SET + LVDESTEN_SET + LVD_DEBOUNCE_SET
    DB      0x1D +CLOCKRATE_SET + HOLD_CK2_SET 
    DB      0x07+ WATCHCLK_SET + RESETPIN_SET + HOLD_CK_SET
    DB      0x5A		
	DB      0xA5	   
	DB      WATCHDOG_SET + CK_FINE_TUNING_SET + ISP_EN_SET + SECURITY_SET

	
END			

