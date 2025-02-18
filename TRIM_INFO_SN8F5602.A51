$NOMOD51
;------------------------------------------------------------------------------
;
;  Copyright (c) 2024 SONiX Technology Co., Ltd.
;  Version 1.0 - SN8F5602, SN8F5601, SN8F56011. Add code option address for MP5
;  *** <<< Use Configuration Wizard in Context Menu >>> ***
;------------------------------------------------------------------------------
;------------------------------------------------------------------------------
;	Inf Block Trim Data Address 
;------------------------------------------------------------------------------	
;  <h> Trim Block
;    <o> IHRC Trim Value <0x000-0x3FF>
;      <i> information address 0x0081, 0x0082 (range: 0x000 - 0x3FF)
IHRCTRIM_SET	EQU		0x080
;    <o> IHRC Rought Trim Value <0x00-0x07>
;      <i> information address 0x0082 (range: 0x00 - 0x07)
IHRCTRIM_RT_SET	EQU		0x03
	
;    <o> LVDIO Trim Value <0x00-0x1F>
;      <i> information address 0x000 (range: 0x00 - 0x1F)
LVDIOTRIM_SET			EQU		0x1A
;    <o> LVDIO Reverse Value <0x00-0x1F>
;      <i> information address 0x0021 (range: 0x00 - 0x1F)
NOTIOTRIM_SET			EQU		0x05
;    <o> LVDNVM Trim Value <0x00-0x0F>
;      <i> information address 0x0022 (range: 0x00 - 0x0F)
LVDNVMTRIM_SET		EQU		0x06
;    <o> LVDNVM Reverse Value <0x00-0x0F>
;      <i> information address 0x0023 (range: 0x00 - 0x0F)
NOTNVMTRIM_SET		EQU		0x09
;    <o> LVDLPM2 Trim Value <0x00-0x0F>
;      <i> information address 0x0024 (range: 0x00 - 0x0F)
LVDLPM2TRIM_SET		EQU		0x06
;    <o> LVDCORE Trim Value <0x00-0x07>
;      <i> information address 0x0025 (range: 0x00 - 0x07)
LVDCORETRIM_SET		EQU		0x06

;    <o> Regulator 1.6V Trim Value <0x00-0x0F>
;      <i> information address 0x0001 (range: 0x00 - 0x0F)
LDO16TRIM_SET			EQU		0x07
;    <o> ADC Trim Value <0x00-0x1F>
;      <i> information address 0x00A0 (range: 0x00 - 0x1F)
ADCTRIM_SET				EQU		0x0F
;    <o> INTREF Trim Value <0x00-0x1F>
;      <i> information address 0x0010 (range: 0x00 - 0x1F)
INTREFTRIM_SET				EQU		0x0F
;    <o> OP Trim Value <0x00-0x7F>
;      <i> information address 0x00C0 (range: 0x00 - 0x7F)
OPTRIM_SET				EQU		0x07	
;    <o> CMP0 Trim Value <0x00-0x7F>
;      <i> information address 0x00C0 (range: 0x00 - 0x7F)
CMP0TRIM_SET				EQU		0x07	
;  </h>

;;------------------------------------------------------------------------------
;;	Information Data Mapping (for engineering)
;;------------------------------------------------------------------------------	
	CSEG	AT		0x4800			;mapping to information 0x0000
		DB			0xFF
		DB		  	LDO16TRIM_SET
		REPT		(32-2)
		DB			0xFF
		ENDM		
		DB			LVDIOTRIM_SET
		DB			NOTIOTRIM_SET
		DB			LVDNVMTRIM_SET
		DB			NOTNVMTRIM_SET
		DB			LVDLPM2TRIM_SET
		DB			LVDCORETRIM_SET
		REPT		(32-6)
		DB			0xFF
		ENDM		

	CSEG	AT		0x4880			;mapping to information 0x0080
		DB      0xFF
		DB      (IHRCTRIM_SET&0xFF)
		DB      ((IHRCTRIM_SET>>8)|(IHRCTRIM_RT_SET<<2))
		REPT		(32-3)				
		DB			0xFF
		ENDM
		DB      ADCTRIM_SET
		DB		INTREFTRIM_SET
	CSEG	AT		0x48C0			;mapping to information 0x00C0			
		DB			OPTRIM_SET
		DB			CMP0TRIM_SET
				
END			

