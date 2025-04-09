// === Start of ADC.c ===
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
#include "ADC.h"
#include "config.h"

/*_____ D E C L A R A T I O N S ____________________________________________*/


/*_____ D E F I N I T I O N S ______________________________________________*/


/*_____ M A C R O S ________________________________________________________*/

/*_____ F U N C T I O N S __________________________________________________*/
void ADC_Init(void)
{
  ADM = 0;
  // 32MHz/4 * 16T = 0.5MHz 
  ADR = ADC_CLK_HOSC | mskGCHS;
  VREFH = VREF_INTERNAL | VREF_VDD;
  ADCAL = SAMPLE_3T5 | ADC_CLK_DIV2;

  ADM |= mskADENB|VOLTAGE_ADC_CHANNEL;
  // ADC calibration
  ADCAL |= mskACS;
  while((ADCAL&mskACS) != 0);
  ADCAL |= mskCALIVALENB;
}



void select_ADC_channel(uint8_t channel) {
  // 設置 ADC 通道選擇寄存器
  ADM =  (ADM & ~0x1F) | channel;
}

void ADC_measure_4_avg(uint8_t channel, uint16_t *result) {
    //uint16_t samples[4];  // 用於存放 4 次量測結果
    uint16_t sum = 0;
    uint8_t i;

    // 切換到指定 ADC 通道
    select_ADC_channel(channel);
  
    // 連續量測 4 次
    for (i = 0; i < 4; i++) {
        START_ADC_CONVERSION;
        while (IS_ADC_FINISH == 0);
        CLEAR_EOC;
        sum += GET_ADC_RESULT;  // 獲取 ADC 結果
    }
    
//    // 找出最大值與最小值並計算總和
//    max_value = samples[0];
//    min_value = samples[0];
//    sum = samples[0];
//    for (i = 1; i < 4; i++) {  // 從索引 1 開始
//        if (samples[i] > max_value) {
//            max_value = samples[i];
//        } else if (samples[i] < min_value) {
//            min_value = samples[i];
//        }
//        sum += samples[i];
//    }
    
    // 去掉最大值與最小值，計算中間兩次的平均值
    //sum = sum - max_value - min_value;
    *result = sum / 4;  // 回傳結果到指定變數位址
}





// === End of ADC.c ===

// === Start of buzzer.c ===
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
#include "buzzer.h"


/*_____ D E C L A R A T I O N S ____________________________________________*/


/*_____ D E F I N I T I O N S ______________________________________________*/


/*_____ M A C R O S ________________________________________________________*/

/*_____ F U N C T I O N S __________________________________________________*/

void Buzzer_Init(void)
{
  BZM = 0;
  
  // 32MHz/65536 = 500Hz
  BZM = BZ_RATE_65536;
}



// === End of buzzer.c ===

// === Start of communication.c ===
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
#include "communication.h"
#include "I2C.h"
#include "system.h"
#include "temperature.h"
#include "config.h"

/*_____ D E F I N I T I O N S ______________________________________________*/


/*_____ D E C L A R A T I O N S ____________________________________________*/
uint8_t i2c_status_code = I2C_STATUS_NORMAL;  // default normal status

/*_____ M A C R O S ________________________________________________________*/

/*_____ F U N C T I O N S __________________________________________________*/
void I2C_Communication(void) {
    static bit f_i2c_transaction_type = 1; // **0: Read, 1: Write**
    static uint8_t i2c_buffer[I2C_BUFFER_SIZE];

    if (cntdown_timer_expired(CNTDOWN_TIMER_I2C)) {
      if (f_i2c_power_received) {
        f_i2c_power_received = 0;
        switch (i2c_buffer[0]) {
          case 0x00: power_setting = 0; break;
          case 0x44: power_setting = 200000; break;
          case 0x48: power_setting = 500000; break;
          case 0x49: power_setting = 800000; break;
          case 0x4A: power_setting = 1000000; break;
          case 0x4B: power_setting = 1300000; break;
          case 0x4C: power_setting = 1600000; break;
          case 0x4D: power_setting = 1800000; break;
          case 0x4E: power_setting = 2000000; break;
          case 0x4F: power_setting = 2200000; break;
          default:   power_setting = 0;       break; // **如果數據無效
        } 
      }        
      
      if (f_i2c_transaction_type) {
            // **準備 NTC 資料**
            i2c_buffer[0] = i2c_status_code;
            i2c_buffer[1] = (uint8_t)TOP_TEMP_C;
            I2C_Write(I2C_SLAVE_ADDRESS, i2c_buffer, 2);
        } else {
            I2C_Read(I2C_SLAVE_ADDRESS, i2c_buffer, 2); // **讀取功率**
        }

        f_i2c_transaction_type = !f_i2c_transaction_type; // **交替讀/寫**
        

        // **重新啟動 43ms 計時**
        cntdown_timer_start(CNTDOWN_TIMER_I2C, I2C_INTERVAL);
    }
}


// === End of communication.c ===

// === Start of comparator.c ===
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
#include "comparator.h"
#include "config.h"
#include "system.h"
#include "power.h"

/*_____ D E C L A R A T I O N S ____________________________________________*/


/*_____ D E F I N I T I O N S ______________________________________________*/


/*_____ M A C R O S ________________________________________________________*/

/*_____ F U N C T I O N S __________________________________________________*/

void Comparator_Init(void)
{
  // CM0 -------------------------------------------------------
  CM0M = 0;
  // P02, P03 switch to analog pin
  P0CON |= (1<<3)|(1<<2); 
  
  CM0M = CM0_FALLING_TRIGGER | CM0_CLK_FCPU;
  CMDB0 = DELAY_4T | DEBOUNCE_10FCPU;
  
  CM0M |= mskCM0EN;
  
  //CM0_IRQ_ENABLE; //HCW** for experiment
  
  // INTERNAL REFERENCE VOLTAGE -------------------------------------------------------
  INTREF = mskINTREFEN | INTREF3V5;
  
  // CM1 -------------------------------------------------------
  // AC Power surge protection @260 Vrms
  // Protection voltage is 260V RMS (peak 367V), 
  // with a resistor divider ratio of 0.0049, resulting in 1.8V.
  
  // After the induction cooker begins operation, the GND potential shifts upward & down, so we need an increase in the voltage surge detection threshold.
  // For this prototype circuit, an upward adjustment of 225mV is required
  CM1M = 0;
  
  // CM1N
  // P04 switch to analog pin
  P0CON |= (1<<4); 
  
  // CM1P
  // 設定 CM1+ 端為內部參考電壓 1.8V + 0.225V = 2.025V
  // INTERNAL 3.5V*37/64 = 2.023V-> CM1RS = 37
  CM1REF = CM1REF_INTREF | 37;
  
  CMDB1 = CM1_DEBOUNCE_8FCPU; // 0.5us
  
  CM1M = mskCM1EN | mskCM1SF |CM1_FALLING_TRIGGER;
  
  IEN2 |= mskECMP1;
  
  // CM2 -------------------------------------------------------
  // IGBT overvoltage protect @1050V (AC power High or Pot be taken off)
  CM2M = 0;
  // CM2N
  // P05 switch to analog pin
  P0CON |= (1<<5);  
  
  // CM2P
  // 1050V*2.5K/827.5K = 3.1722V
  // INTERNAL 3.5V*58/64 = 3.1718V -> CM2RS = 58 
  CM2REF = CM2REF_INTREF | 58; 
  
  //CM2REF = CM2REF_INTREF | 38;  //700V
  //CM2REF = CM2REF_INTREF | 50;  //900V
  
  
  CM2M = mskCM2EN | mskCM2SF |CM2_FALLING_TRIGGER;
  
  IEN2 |= mskECMP2;
  
  // CM3 -------------------------------------------------------
  // AC Power Sync.
  CM3M = 0;
  
  // CM3P
  // P04 switch to analog pin
  // P0CON |= (1<<4);    // It has already been executed during CM2 initialization.
  
  // CM3N
  // Set CM3- to internal reference voltage 0.5V
  // INTERNAL 3.5V*9/64 = 0.5V (0.49V) -> CM3RS = 9 
  CM3REF = CM3REF_INTREF | 9;
  
  //CM3REF = CM3REF_INTREF | 1; // 54mV
  
  CMDB3 = CM3_DEBOUNCE_20FCPU;
  
  CM3M = mskCM3EN | CM3_FALLING_TRIGGER;
  
  IEN2 |= mskECMP3;
  
  // CM4 -------------------------------------------------------
  // Current Surge Protection 
  // Configured for around 10 A
  
  // For the prototype, the amplification ratio is approximately 47K/1.25K = 37.6x. With a 10A current + 0.01Ω constantan wire, the output is:
  // 0.1V × 37.6 = 3.76V. Adding the OP offset base current (~0.385V), 
  // the total becomes:3.76V + 0.385V = 4.14V
  
  CM4M = 0;
  // CM4N from OPO
  
  // CM4P
  // VDD 5V*48/64 = 3.8V -> CM4RS = 48 
  //CM4REF = CM4REF_VDD | 48;
  
  // VDD 5V*53/64 = 4.14V -> CM4RS = 53 
  //CM4REF = CM4REF_VDD | 53;
  
  // VDD 5V*57/64 = 4.5V -> CM4RS = 57
  CM4REF = CM4REF_VDD | 57; 
  
  // VDD 5V*61/64 = 4.8V -> CM4RS = 61 
  //CM4REF = CM4REF_VDD | 61; 
  
  CM4M = mskCM4SF | CM4_FALLING_TRIGGER | CM4N_OPO;
  
  IEN2 |= mskECMP4;
}

void Surge_Protection_Modify(void)
{
  // Since the op-amp's trim value is reduced by 6 (due to die-to-die variations), 
  // the actual measured base_current must be compared with the default base_current 
  // to properly adjust the CMP4 protection threshold
  
  //TBD...
  
  // Enable CM4
  CM4M |= mskCM4EN;
}



void comparator0_ISR(void) interrupt ISRCmp0
{
  //P10 = 1; //HCW**
  
  if(pot_pulse_cnt < 250)
  {
    pot_pulse_cnt++; // 每次中斷觸發，脈衝計數器自增
  }    
  
  //P10 = 0; //HCW**
 
    // 其他 Comparator0 的邏輯可在此處添加
}

void comparator1_ISR(void) interrupt ISRCmp1
{  
  // 當 CM1 觸發中斷，代表電壓浪湧發生，立起 Surge_Overvoltage_Flag
  Surge_Overvoltage_Flag  = 1;
  
  // In interrupt, simply stop the heating logic
  P01 = 1;  //PWM Pin
  PW0M = 0;
}

void comparator2_ISR(void) interrupt ISRCmp2
{
  //P10 = 1; //HCW**
  
  // Patch for CM2SF can't be triggered by edge.
  if ((PW0D >= (3 + PWM_MIN_WIDTH))) { // 確保不小於最小值
      PW0D -= 1;
  } else {
      PW0D = PWM_MIN_WIDTH; // 避免低於最小寬度
  }
  
  //P10 = 0; //HCW**
}

#define AC_SYNC_DEBOUNCE_TICKS 32  // **AC 週期同步去抖動時間 32*125us (4ms)**
/*
   AC_SYNC_DEBOUNCE_TICKS is used to ensure that `f_CM3_AC_sync` is only triggered at the start of a new AC cycle:
   1. When `CM3+` is lower than `CM3-` (0.5V), `CM3_ISR` will be triggered, but there may be signal jitter.
   2. This variable sets `4ms (32 system_ticks)` as the debounce time.
   3. `f_CM3_AC_sync` is only allowed to be triggered again after `AC_SYNC_DEBOUNCE_TICKS`, preventing jitter from affecting the timing accuracy.
*/

volatile bit f_CM3_AC_sync = 0;
volatile uint8_t idata CM3_AC_sync_cnt = 0;
volatile uint8_t idata CM3_last_sync_tick = 0; // Record the time when `f_CM3_AC_sync` was set to `1`

void comparator3_ISR(void) interrupt ISRCmp3
{
  // **Ensure that `f_CM3_AC_sync` is only triggered within a new AC cycle**
  if ((uint8_t)(system_ticks - CM3_last_sync_tick) >= AC_SYNC_DEBOUNCE_TICKS) {
    f_CM3_AC_sync = 1;       // **Mark the start of a new AC cycle**
    CM3_AC_sync_cnt++;
    CM3_last_sync_tick = system_ticks; // **Update `last_sync_tick`**
  }
}

void comparator4_ISR(void) interrupt ISRCmp4
{  
  // 當 CM4 觸發中斷，代表電流浪湧發生，立起 Surge_Overcurrent_Flag
  Surge_Overcurrent_Flag  = 1;
  
  // In interrupt, simply stop the heating logic
  P01 = 1;  //PWM Pin
  PW0M = 0;
}

// === End of comparator.c ===

// === Start of gpio.c ===
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
#include "gpio.h"


/*_____ D E C L A R A T I O N S ____________________________________________*/


/*_____ D E F I N I T I O N S ______________________________________________*/


/*_____ M A C R O S ________________________________________________________*/

/*_____ F U N C T I O N S __________________________________________________*/

void GPIO_Init(void)
{
  // P00 Fan(BUZZ)  | In
  // P01 PWM        | In
  // P02 CM0P       | In
  // P03 CM0N       | In
  // P04 CM1N       | In
  // P05 CM2N       | In
  // P06 No sue     | OUT L
  // P07 No sue     | OUT L
  P0 = (1<<1);  // P01 H (P01 always output)
  P0M = (1<<7)|(1<<6)|(1<<0); //HCW** P00 debug
  P0CON = (1<<5)|(1<<4)|(1<<3)|(1<<2);  //P02 P03 P04 P05
  
  // P10 No sue     | Out L
  // P11 IGBTT      | In
  // P12 TOPT       | In
  // P13 SDA        | In
  // P14 SCL        | In
  // P15 No sue     | Out L
  // P16 No sue     | Out L
  P1 = 0;
  P1M = (1<<6)|(1<<5)|(1<<0);
  P1CON = (1<<2)|(1<<1);  //P12 P11
  
  // P40 OPO        | In
  // P41 OPN        | In
  // P42 DC_PWR_V   | In
  P4 = 0;
  P4M = 0;
  P4CON = (1<<2)|(1<<1)|(1<<0);  //P42 P41 P40
  
}



// === End of gpio.c ===

// === Start of I2C.c ===
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
#include "I2C.h"

/*_____ D E F I N I T I O N S ______________________________________________*/
#define SLAVE_ADDRESS  0x55  // **Slave 位址**

#define I2C_WRITE_MODE  0
#define I2C_READ_MODE   1



/*_____ D E C L A R A T I O N S ____________________________________________*/
volatile uint8_t* I2C_Buf; 
uint8_t I2C_Slave_Address;
uint8_t I2C_Length;
volatile uint8_t I2C_Index;
volatile bit I2C_Mode;
volatile bit f_i2c_power_received = 0;   // **功率接收完成標誌**

/*_____ M A C R O S ________________________________________________________*/

/*_____ F U N C T I O N S __________________________________________________*/
void I2C_Init(void)
{
  I2CCON = 0;
  
  /*	Configure the I2C's IO status and register */
  I2C_PIN_SET_INPUT_1;
  SMBSEL = mskI2CMX;
  
  // I2C 32MHz/320 = 100KHz
  I2CCON = mskI2CCON_I2C_ENABLE | mskI2CCON_I2C_DIV_320;
  
  /*	Enable I2C interrupt  */
  IEN1 = mskIEN1_INT_I2C;
}

uint8_t I2C_Calculate_Checksum(uint8_t *databuf, uint8_t length) {
    uint8_t i, sum = 0;
    for (i = 0; i < length; i++) {
        sum += databuf[i];
    }
    return ~sum; // **反向總和**
}


void I2C_Write(uint8_t slave_addr, uint8_t *databuf, uint8_t length) {
    // **計算並附加 `Checksum`**
    databuf[length] = I2C_Calculate_Checksum(databuf, length);
    length++;  // **包含 `Checksum`**

    // **設定 I2C 傳輸參數**
    I2C_Slave_Address = slave_addr;
    I2C_Buf = databuf;
    I2C_Length = length;
    I2C_Index = 0;
    I2C_Mode = I2C_WRITE_MODE;

    // **開始 I2C 傳輸**
    I2CCON |= mskI2CCON_I2C_STA; // **啟動 I2C**
}
void I2C_Read(uint8_t slave_addr, uint8_t *databuf, uint8_t length) {
    // **設定 I2C 讀取參數**
    I2C_Slave_Address = slave_addr;
    I2C_Buf = databuf;
    I2C_Length = length;
    I2C_Index = 0;
    I2C_Mode = I2C_READ_MODE;

    // **開始 I2C 讀取**
    I2CCON |= mskI2CCON_I2C_STA; // **啟動 I2C**
}

void I2C_ISR(void) interrupt ISRI2c {
    switch (I2CSTA) {
        case 0x08: // **Start Condition Transmitted**
        case 0x10: // **Repeated Start**
            I2CDAT = (I2C_Slave_Address << 1) | I2C_Mode;
            break;

        case 0x18: // **Address ACKed (TX Mode)**
        case 0x28: // **Data ACKed (TX Mode)**
            I2CCON &= ~mskI2CCON_I2C_STA;
        
            if (I2C_Index < I2C_Length) {
              I2CDAT = I2C_Buf[I2C_Index++];
            } else {
              I2CCON &= ~mskI2CCON_I2C_STA;
              I2CCON |= mskI2CCON_I2C_STO;  // **發送 Stop**
              I2C_Index = 0;
            }
            break;

        case 0x20: // **Address NACK (TX Mode)**
        case 0x30: // **Data NACK (TX Mode)**
            I2CCON &= ~mskI2CCON_I2C_STA;
            I2CCON |= mskI2CCON_I2C_STO;  // **發送 Stop**
            I2C_Index = 0;
            break;

        case 0x40: // **Address ACKed (RX Mode)**
            I2CCON &= ~mskI2CCON_I2C_STA;
        
            if (I2C_Index < I2C_Length - 1) {
                I2CCON |= mskI2CCON_I2C_AA;  // **繼續接收**
            } else {
                I2CCON &= ~mskI2CCON_I2C_AA; // **最後一個字節**
            }
            break;
        
        case 0x48:      /*  after addr_NACK, repeat start condition */
            I2CCON |= mskI2CCON_I2C_STA;
            I2CCON |= mskI2CCON_I2C_STO;
            break;
        case 0x50: // **Data ACKed (RX Mode)**
            I2C_Buf[I2C_Index++] = I2CDAT;
            if (I2C_Index < I2C_Length - 1) {
                I2CCON |= mskI2CCON_I2C_AA;  // **繼續接收**
            } else {
                I2CCON &= ~mskI2CCON_I2C_AA; // **最後一個字節**
            }
            break;

        case 0x58: // **Data NACKed (RX Mode)**
            I2C_Buf[I2C_Index++] = I2CDAT;
            I2CCON &= ~mskI2CCON_I2C_AA;
            I2CCON |= mskI2CCON_I2C_STO;  // **發送 Stop**
            I2C_Index = 0;
            // **驗證 Checksum**
            if (I2C_Buf[1] == ~(I2C_Buf[0])) {  
                f_i2c_power_received = 1;  // **標記功率接收完成**
            }
            break;

        case 0xF8: // **Bus Error**
            I2CCON |= mskI2CCON_I2C_STO;  // **發送 Stop**
            I2C_Index = 0;
            break;
        
        default:				
            I2CCON &= ~mskI2CCON_I2C_STA;
            I2CCON |= mskI2CCON_I2C_STO;
            break;
    }

    // **清除 I2C 旗標**
    I2CCON &= ~mskI2CCON_I2C_FLAG;
}


// === End of I2C.c ===

// === Start of Init.c ===
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
// === End of Init.c ===

// === Start of main.c ===
/******************** (C) COPYRIGHT 2024 SONiX *******************************
* COMPANY:	SONiX
* DATE:		  2025/01
* AUTHOR:		HCW
* IC:			  SR56F27
*____________________________________________________________________________
* REVISION		Date				User		Description
* 0.1         2025/02 
*____________________________________________________________________________
*****************************************************************************/

/*_____ I N C L U D E S ____________________________________________________*/
#define __XRAM_SFR_H__
#include <SN8F5602.h>
#include "timer0.h"
#include "comparator.h"
#include "OP_amp.h"
#include "gpio.h"
#include "ADC.h"
#include "PWM.h"
#include "system.h"
#include "communication.h"
#include "power.h"
#include "temperature.h"
#include "buzzer.h"
#include "I2C.h"

/*_____ D E F I N I T I O N S ______________________________________________*/


#define SYSTEM_TICKS_PER_10MS 80  // 每 10 ms 的計數 (125 μs * 80 = 10 ms)
#define SYSTEM_10MS_PER_SECOND 100  // 每秒包含的 10 ms 計數

TaskType current_task = TASK_HEAT_CONTROL; // 當前任務

/*_____ D E C L A R A T I O N S ____________________________________________*/

/*_____ M A C R O S ________________________________________________________*/

/*_____ F U N C T I O N S __________________________________________________*/
void Warmup_Delay(void);

/*****************************************************************************
* Function		: main
* Description	: Test loop
* Input			: None
* Output		: None
* Return		: None
* Note			: None
*****************************************************************************/
void main (void)
{
  //DEGCMD = 0x00; // Disable SWAT pin
  #if TUNE_MODE == 1
  OPM = 0x01;
  OPCAL -= 6; 
  #endif
  
  // 初始化模塊
  SystemCLK_Init();       // 初始化 系統頻率
  GPIO_Init();            // 初始化 GPIO 配置
  Timer0_Init();          // 初始化 Timer0，用於產生 125 微秒中斷
  Comparator_Init();      // 初始化所有比較器 (CM0, CM1, CM2)
  OP_Amp_Init();          // 初始化運算放大器，用於電流量測
  PWM_Init();             // 初始化PWM for IGBT driving
  Buzzer_Init();          // 初始化風扇驅動
  I2C_Init();             // 初始化 I2C 接口
  
  CNTdown_Timer_Init();   // 初始化倒數計時模組
  
  EAL = 1;                // 啟用全域中斷
  
  Warmup_Delay();         // 30ms
  
  ADC_Init();             // 初始化 ADC，用於多通道量測
  
  Measure_AC_Low_Time();  // 量測AC low tume, 用於IGBT C級能量漸放時間 & 啟動間歇加熱時間點
  Detect_AC_Frequency();  // 50Hz or 60Hz
  Measure_Base_Current(); // 量測Base電流, for OP offset
  Surge_Protection_Modify(); 
  
  // 設置初始系統狀態
  system_state = STANDBY;  // 系統默認進入待機模式

  #if TUNE_MODE == 1
//  power_setting = 800000;
  #endif
  
  // f_125us reset to zero
  f_125us = 0;
  
  // 進入主程式循環
  while (1) {
    WDTR = 0x5A; // Clear watchdog
    I2C_Communication();
    
    // 125 μs 定時邏輯
    if (f_125us) {
      f_125us = 0;  // 清除 125 微秒旗標
            
      // 更新系統時間
      Update_System_Time();  
      
      // 獨立執行任務
      Power_read();
          
      // 任務循環
      switch (current_task) {
        case TASK_HEAT_CONTROL:
            Heat_Control();
            current_task = TASK_POWER_CONTROL; // 切換到下一個任務
            break;
        
        case TASK_POWER_CONTROL:
            Power_Control();
            current_task = TASK_QUICK_CHANGE_DETECT; // 切換到下一個任務
            break;  
        
        case TASK_QUICK_CHANGE_DETECT:
            Quick_Change_Detect();
            current_task = TASK_TEMP_MEASURE; // 切換到下一個任務
            break;

        case TASK_TEMP_MEASURE:
            Temp_Measure();
            current_task = TASK_TEMP_PROCESS; // 切換到下一個任務
            break;
          
        case TASK_TEMP_PROCESS:
            Temp_Process();
            current_task = TASK_CURRENT_POT_CHECK; // 切換到下一個任務
            break;

        case TASK_CURRENT_POT_CHECK:
            Pot_Detection_In_Heating();  // 執行電流檢鍋任務
            current_task = TASK_SHUTDOWN; // 切換到下一個任務
            break;

        case TASK_SHUTDOWN:
            //Shutdown_Task();  // 執行關機任務
            current_task = TASK_ERROR_PROCESS; // 切換到下一個任務
            break;

        case TASK_ERROR_PROCESS:
            Error_Process();  // 處理錯誤任務
            current_task = TASK_HEAT_CONTROL;  // 循環回到第一個任務
            break;

        default:  break;
      }
    } //(f_125us) end
    
    // **間歇加熱模式：即時檢查 AC 訊號變化**
    Periodic_Power_Control();
    
  } //while end
 
}

void Warmup_Delay(void)
{
  uint8_t cnt = 0;
  WDTR = 0x5A; // Clear watchdog
  
  while(cnt < 240) //125us* 240 = 30ms
  {    
    if (f_125us)
    {
      f_125us = 0;
      cnt++;
    }
  }
  
}

// === End of main.c ===

// === Start of OP_amp.c ===
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
#include "OP_amp.h"


/*_____ D E C L A R A T I O N S ____________________________________________*/


/*_____ D E F I N I T I O N S ______________________________________________*/


/*_____ M A C R O S ________________________________________________________*/

/*_____ F U N C T I O N S __________________________________________________*/

void OP_Amp_Init(void)
{
  OPM = 0;
  // P40, P41 switch to analog pin
  P4CON |= (1<<1)|(1<<0); 
  
  OPM = OPN_IN_SHORT | mskOPOEN | OPP_GND;
  // Gain 
  OPGS = 0; // x1
  
  OPM |= mskOPEN;
}



// === End of OP_amp.c ===

// === Start of power.c ===
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
#include "power.h"
#include "system.h"
#include "comparator.h"
#include "PWM.h"
#include "ADC.h"
#include "buzzer.h"
#include <math.h>

/*_____ D E F I N I T I O N S ______________________________________________*/


/*_____ D E C L A R A T I O N S ____________________________________________*/
bit f_heating_initialized = 0;     // 加熱功能是否已初始化
bit f_power_updated = 0;

uint8_t level = 0;
uint16_t voltage_adc_new = 0;  // Store the measured voltage value (adc_code)
uint16_t current_adc_new = 0;  // Store the measured current value (adc_code)
uint16_t PW0D_backup = 0;
static uint8_t pwr_read_cnt = 0;      // Number of measurements during heating
static uint32_t current_adc_sum = 0;
static uint32_t voltage_adc_sum = 0;



/*_____ M A C R O S ________________________________________________________*/

/*_____ F U N C T I O N S __________________________________________________*/
void measure_power_signals(void);
void stop_heating(void);
void pause_heating(void);
void IGBT_C_slowdown(void);
void shutdown_process(void);
void finalize_shutdown(void);
uint16_t current_lookup_interpolation(uint16_t adc_val);

void Power_read(void)
{
  static uint16_t current_adc_avg = 0;
  static bit last_power_measure_valid = 0;   // Track previous state of f_power_measure_valid
 
  // Measure the system current & voltage
  ADC_measure_4_avg(CURRENT_ADC_CHANNEL, &current_adc_new);
  ADC_measure_4_avg(VOLTAGE_ADC_CHANNEL, &voltage_adc_new);

  // Handle PERIODIC_HEATING mode separately
  if (system_state == PERIODIC_HEATING) {
    if (last_power_measure_valid != f_power_measure_valid) {
      reset_power_read_data();
    }
    last_power_measure_valid = f_power_measure_valid;  // Update state tracking
  }
  
  // Accumulate Data
  current_adc_sum += current_adc_new;
  voltage_adc_sum += voltage_adc_new;
  pwr_read_cnt++;
 
  // Calculate power when reaching a full AC cycle
  if (pwr_read_cnt >= measure_per_AC_cycle)
  {
    if(f_AC_50Hz){
      //voltage_RMS_V = voltage_adc_sum/MEASUREMENTS_PER_50HZ(160)/4096*5*(1/0.0163)*1.11; // V (->avg->RMS)
      //"0.0163" comes from voltage divider calculation.
      //For a full-wave rectified sine wave, Form Factor(K) = 1.11, meaning Vrms is 1.11*Vavg .
      voltage_RMS_V = (voltage_adc_sum*2179)>>22;
      
      //current_adc_avg = current_adc_sum/MEASUREMENTS_PER_50HZ(160);  // (->avg)
      current_adc_avg = (current_adc_sum * 3277) >> 19;
    }else{
      //voltage_RMS_V = voltage_adc_sum/MEASUREMENTS_PER_60HZ(133)/4096*5*(1/0.0163)*1.11; // V (->avg->RMS)
      //"0.0163" comes from voltage divider calculation.
      //For a full-wave rectified sine wave, Form Factor(K) = 1.11, meaning Vrms is 1.11*Vavg .
      voltage_RMS_V = (voltage_adc_sum*5243)>>23;
      
      //current_adc_avg = current_adc_sum/MEASUREMENTS_PER_60HZ(133); // (->avg)
      current_adc_avg = (current_adc_sum * 1971) >> 18;
    }
    
    // Remove current_base
    if(current_adc_avg > current_base){
      current_adc_avg -= current_base; 
    }else{
      current_adc_avg = 0;
    }
    
    // mA (->RMS)
    current_RMS_mA = current_lookup_interpolation(current_adc_avg);
    //
    if (f_power_measure_valid) {
      // Caculate power
      current_power = (uint32_t)voltage_RMS_V * current_RMS_mA;
      f_power_updated = 1;
    }
    
    #if TUNE_MODE == 1
//    tune_cnt++;
//    if(tune_cnt >= 300)
//    {
//      PW0M = 0;
//      while(1);
//    }
    
//    tune_record1 = current_RMS_mA;
//    tune_record2 = current_adc_avg;
    #endif
    
    pwr_read_cnt = 0;
    current_adc_sum = 0;
    voltage_adc_sum = 0;
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void measure_power_signals(void)
{
  // Measure system current using ADC
  ADC_measure_4_avg(CURRENT_ADC_CHANNEL, &current_adc_new);
  
  // Measure AC voltage using ADC
  ADC_measure_4_avg(VOLTAGE_ADC_CHANNEL, &voltage_adc_new);

  // **Only accumulate the measured values when heating is active**
  if (f_heating_initialized) {
    current_adc_sum += current_adc_new;
    voltage_adc_sum += voltage_adc_new;
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 * @brief Resets internal variables used in power reading.
 *
 * This clears the ADC accumulation counters and sample count
 * used by Power_read(), typically called at startup or heating init.
 */
void reset_power_read_data(void)
{
  f_power_updated = 0;
  pwr_read_cnt = 0;
  current_adc_sum = 0;
  voltage_adc_sum = 0;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
typedef struct {
    uint16_t adc;     // ADC valuw without current_base
    uint16_t current; // 對應電流值 (單位：mA)
} LookupEntry;

#define TABLE_SIZE 9
// ADC_Code(base)
const LookupEntry code lookupTable[TABLE_SIZE] = {
    {0,    6},
    {500,    1813},
    {1000,   3620},
    {1500,   5428},
    {2000,   7235},
    {2500,   9042},
    {3000,   10849},
    {3500,   12656},
    {4000,   14464}
};
// To save computational resources on an 8-bit MCU, we use fixed-point arithmetic.
// We choose SCALE_BITS = 8 (i.e., multiply by 256).
// The fixed slope is 3.614397; its fixed-point representation is: 3.614397 * 256 ≈ 925.
#define SCALE_BITS 8
#define FIXED_SLOPE 925

// current_lookup_interpolation uses the pre-calculated fixed slope for interpolation.
// It takes an ADC value as input and returns the corresponding current value (in IRMS mA).
uint16_t current_lookup_interpolation(uint16_t adc_val) 
{
  uint8_t i;
  uint16_t adc_low, current_low, diff;
  uint16_t interpolated_current;
  
  // If ADC value is below the minimum in the table, return the minimum current.
  if (adc_val <= lookupTable[0].adc)
      return lookupTable[0].current;
  
  // If ADC value is above the maximum in the table, return the maximum current.
  if (adc_val >= lookupTable[TABLE_SIZE - 1].adc)
      return lookupTable[TABLE_SIZE - 1].current;

  // Find the interval in which adc_val falls
  for (i = 0; i < TABLE_SIZE - 1; i++) {
    if (adc_val < lookupTable[i+1].adc) {
      adc_low = lookupTable[i].adc;
      current_low  = lookupTable[i].current;
      diff = adc_val - adc_low;
      // interpolated_current = current_low  + (diff * FIXED_SLOPE) / 256
      interpolated_current = current_low  + (((uint32_t)diff * FIXED_SLOPE) >> SCALE_BITS);
      return interpolated_current;
    }
  }
  
  // Theoretically should not reach here.
  return lookupTable[TABLE_SIZE - 1].current;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#define DEFAULT_BASE_CURRENT_ADC   316   // 預設基準電流 ADC 值
#define BASE_CURRENT_TOLERANCE     8   // 允許誤差百分比（8%）

// 在編譯期間計算允許的上下限
#define BASE_CURRENT_LOWER_LIMIT  (((uint16_t)DEFAULT_BASE_CURRENT_ADC * (100 - BASE_CURRENT_TOLERANCE)) / 100)
#define BASE_CURRENT_UPPER_LIMIT  (((uint16_t)DEFAULT_BASE_CURRENT_ADC * (100 + BASE_CURRENT_TOLERANCE)) / 100)


uint16_t current_base = 0;

void Measure_Base_Current(void) {
  uint16_t measured_value;

  // 量測 ADC 基準電流值
  ADC_measure_4_avg(CURRENT_ADC_CHANNEL, &measured_value);
  
  // 檢查是否超出允許範圍
  if (measured_value < BASE_CURRENT_LOWER_LIMIT || measured_value > BASE_CURRENT_UPPER_LIMIT) {
      current_base = DEFAULT_BASE_CURRENT_ADC;  // 超出範圍則使用預設值
  } else {
      current_base = measured_value;  // 正常範圍內則使用測得值
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// 定義電壓和電流快速變化及上下限
#define VOLTAGE_UPPER_LIMIT          255     // V 過壓觸發
#define VOLTAGE_RECOVER_HIGH         245     // V 過壓回復

#define VOLTAGE_LOWER_LIMIT          170     // V 欠壓觸發
#define VOLTAGE_RECOVER_LOW          180     // V 欠壓回復

#define CURRENT_UPPER_LIMIT_mA      11000   // 11A = 11000mA
#define CURRENT_RECOVER_LIMIT_mA    10000   // 10A 過流回復

//#define VOLTAGE_CHANGE_THRESHOLD 20    // 電壓快速變化閾值 (單位：伏特)
//#define CURRENT_CHANGE_THRESHOLD 10    // 電流快速變化閾值 (單位：安培)

void Quick_Change_Detect() {
  // 前一次的測量值
//  static uint16_t last_voltage = 0;     // 上次測量的電壓值
//  static uint16_t last_current = 0;     // 上次測量的電流值  
  
    // === 過壓檢查與回復 ===
    if (error_flags.f.Over_voltage) {
        if (voltage_RMS_V < VOLTAGE_RECOVER_HIGH) {
            error_flags.f.Over_voltage = 0;
        }
    } else if (voltage_RMS_V > VOLTAGE_UPPER_LIMIT) {
        error_flags.f.Over_voltage = 1;
        // Simply stop the heating logic
        P01 = 1;  //PWM Pin
        PW0M = 0;
    }
  
    // === 欠壓檢查與回復 ===
    if (error_flags.f.Low_voltage) {
        if (voltage_RMS_V > VOLTAGE_RECOVER_LOW) {
            error_flags.f.Low_voltage = 0;
        }
    } else if (voltage_RMS_V < VOLTAGE_LOWER_LIMIT) {
        error_flags.f.Low_voltage = 1;
        // Simply stop the heating logic
        P01 = 1;  //PWM Pin
        PW0M = 0;
    }
    
    // === 過電流檢查與回復 ===
    if (error_flags.f.Over_current) {
        if (current_RMS_mA < CURRENT_RECOVER_LIMIT_mA) {
            error_flags.f.Over_current = 0;
        }
    } else if (current_RMS_mA > CURRENT_UPPER_LIMIT_mA) {
        error_flags.f.Over_current = 1;
        // Simply stop the heating logic
        P01 = 1;  //PWM Pin
        PW0M = 0;
    }
    
//    // 檢查電壓是否快速變化
//    if (abs(voltage_IIR_new - last_voltage) > VOLTAGE_CHANGE_THRESHOLD) {
//        error_flags.f.Voltage_quick_change = 1;  // 電壓快速變化
//    } else {
//        error_flags.f.Voltage_quick_change = 0;  // 清除快速變化標誌
//    }

//    // 檢查電流是否快速變化
//    if (abs(current_RMS_mA - last_current) > CURRENT_CHANGE_THRESHOLD) {
//        error_flags.f.Current_quick_large = 1;  // 電流快速變化
//    } else {
//        error_flags.f.Current_quick_large = 0;  // 清除快速變化標誌
//    }

//    // 更新上次的測量值
//    last_voltage = voltage_RMS_V;
//    last_current = current_RMS_mA;
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void stop_heating(void)
{
  // 實現停止加熱邏輯
  P01 = 1;  //PWM Pin
  PW0M = 0;
  CM0M &= ~mskCM0SF;        // Patch: Disable CM0 pulse trigger, if enable pulse trigger PGOUT can't trigger.
  
  f_pot_detected = 0;       // 重置鍋具檢測標誌
  f_heating_initialized = 0;
}

void pause_heating(void)
{
  // 暫停加熱
  PW0M &= ~(mskPW0EN|mskPW0PO|mskPWM0OUT); // Disable PWM / pulse / normal PWM function
  CM0M &= ~mskCM0SF;            // Patch: Disable CM0 pulse trigger, if enable pulse trigger PGOUT can't trigger.
  f_heating_initialized = 0;    // Clear加熱已初始化標誌
}

void shutdown_process(void)
{
    // 一次性停止加熱並保存狀態
    stop_heating();
    //save_system_state();
}
void finalize_shutdown(void)
{
  _nop_(); //HCW**

}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
typedef struct {
    uint8_t heat_cycle;  // 加熱週期數
    uint8_t rest_cycle;  // 休息週期數
} PeriodicConfig;

const PeriodicConfig code Periodic_table[] = {
    {8, 2}, // level 0: 加熱 8 週期，休息 2 週期（對應 800000mW）
    {5, 5}, // level 1: 加熱 5 週期，休息 5 週期（對應 500000mW）
    {4, 16}  // level 2: 加熱 2 週期，休息 8 週期（對應 200000mW）
};

void Heat_Control(void)
{
  static uint8_t prev_level = 0xFF;  // // For PERIODIC_HEATING, use invalid default to ensure first-time reset
  
  // 如果系統狀態為 ERROR
  if (system_state == ERROR) {
      return;
  }

  // 如果功率設定為 0，直接切換至待機狀態
  if (power_setting == 0) {
      system_state = STANDBY; // 切換系統狀態為待機
      stop_heating(); // 停止加熱
      return;
  }

  // 檢查鍋具狀態
  if (f_pot_detected == 0) {
    // 啟動Fan
    BUZZER_ENABLE;
    
    Pot_Detection(); // 執行鍋具檢測邏輯
    return;
  }
  
  // 一般加熱模式
  if (power_setting > 800000) {     
    target_power = power_setting; // 設定為實際功率
    system_state = HEATING;       // 切換系統狀態為 HEATING
    
    if (f_heating_initialized) {
      f_power_measure_valid = 1;
    }
  }
  else // 間歇加熱模式
  {
    switch (power_setting) {    // 檔位判斷
      case 800000: level = 0; break; // 1檔
      case 500000: level = 1; break; // 2檔
      case 200000: level = 2; break; // 3檔
      default: return; // 確保安全
    }
    // Reset AC sync counter if level has changed
    if (level != prev_level)
    {
      periodic_AC_sync_cnt = 0;
      prev_level = level;
    }
    
    target_power = PERIODIC_TARGET_POWER;
    system_state = PERIODIC_HEATING; // 切換系統狀態為間歇加熱模式
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
typedef enum {
    PERIODIC_HEAT_PHASE,
    PERIODIC_HEAT_END_PHASE,  // **確保最後一個加熱週期結束後，先等 `ac_half_low_ticks_avg`**
    PERIODIC_REST_PHASE,
    PERIODIC_SLOWDOWN_PHASE
} PeriodicHeatState;

uint8_t periodic_AC_sync_cnt = 0;
bit f_power_measure_valid  = 0;  // **Flag to indicate current measurement stabilization**

void Periodic_Power_Control(void) {
  static PeriodicHeatState periodic_heat_state = PERIODIC_REST_PHASE;
  static uint8_t phase_start_tick = 0;  // **共用變數：記錄 SLOWDOWN_PHASE & HEAT_END_PHASE 開始時間*
  static bit f_first_entry = 1;  // **標記是否為第一次進入 PERIODIC_HEATING**
  static bit f_periodic_pulse_init = 1;  // 標記是否需要 PULSE_WIDTH_PERIODIC_START
  uint8_t elapsed_ticks;
  
  // **確保只有在 PERIODIC_HEATING 狀態下執行**
  if (system_state != PERIODIC_HEATING) {
      f_first_entry = 1;  // **當離開 PERIODIC_HEATING 時，重置標誌**
      f_periodic_pulse_init = 1;  // **當離開 PERIODIC_HEATING 時，重置標誌**
      return;
  }
  
  // **初始化狀態，僅在進入 PERIODIC_HEATING 的第一個周期執行**
  if (f_first_entry) {
    periodic_AC_sync_cnt = 0;
    f_power_measure_valid = 0;
    
    // **判斷是否已經初始化過加熱**
    if (f_heating_initialized) {
        periodic_heat_state = PERIODIC_HEAT_PHASE;
    } else {
        periodic_heat_state = PERIODIC_REST_PHASE;
    }
    
    f_first_entry = 0;  // clear f_first_entry
  }  
  
  // **檢查是否有新的 AC 訊號週期**
  if (f_CM3_AC_sync) {
      f_CM3_AC_sync = 0;
      periodic_AC_sync_cnt++;
  }
  
  // **計算經過時間**
  elapsed_ticks = system_ticks - phase_start_tick;

  switch (periodic_heat_state) {
    case PERIODIC_REST_PHASE:
      if (periodic_AC_sync_cnt >= Periodic_table[level].rest_cycle) {
        periodic_AC_sync_cnt = 0;
        
        // Backup PW0D before IGBT_C_slowdown
        PW0D_backup = PW0D;
        IGBT_C_slowdown();
        phase_start_tick = system_ticks;
        periodic_heat_state = PERIODIC_SLOWDOWN_PHASE;
      }
      break;

    case PERIODIC_SLOWDOWN_PHASE:
      // **等待 `ac_half_low_ticks_avg` 週期時間，再進入加熱**
      if (elapsed_ticks >= ac_half_low_ticks_avg) {
        pause_heating();    // Stop IGBT_C_slowdown PWM
        
        // Restore PW0D after IGBT_C_slowdown
        PW0D = PW0D_backup;
        
        init_heating(HEATING_IMMEDIATE, f_periodic_pulse_init ? PULSE_WIDTH_PERIODIC_START : PULSE_WIDTH_NO_CHANGE);
        f_periodic_pulse_init = 0;
        
        periodic_heat_state = PERIODIC_HEAT_PHASE;
      }
      break;

    case PERIODIC_HEAT_PHASE:
      if (periodic_AC_sync_cnt == 2) {
        f_power_measure_valid = 1;
      }
    
      if (periodic_AC_sync_cnt >= Periodic_table[level].heat_cycle) {
        periodic_AC_sync_cnt = 0;
        phase_start_tick = system_ticks;
        periodic_heat_state = PERIODIC_HEAT_END_PHASE;
      }
      break;

    case PERIODIC_HEAT_END_PHASE:
      // **等待 `ac_half_low_ticks_avg`，才真正停止加熱**
      if (elapsed_ticks >= ac_half_low_ticks_avg) {
        pause_heating();
        f_power_measure_valid = 0;
        periodic_heat_state = PERIODIC_REST_PHASE;
      }
      break;
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#define POT_HEATING_CURRENT_DELAY_MS 16  // Delay pot detection after heating starts

void init_heating(uint8_t sync_ac_low, PulseWidthSelect pulse_width_select)
{ 
  // Wait for AC low if required
  if (sync_ac_low) {
    f_CM3_AC_sync = 0; // Clear AC sync flag before waiting
    while (!f_CM3_AC_sync); // Block execution until AC low is detected
  }
  
  // Patch: Protect for PWM accidentally switch mode
  PW0M &= ~mskPW0EN;    // Disable PWM
  CM0M &= ~mskCM0SF;    // Disable CM0 pulse trigger
  
  // Set PWM duty cycle based on pulse_width_select
  switch (pulse_width_select) {
    case PULSE_WIDTH_MIN:  // Set PW0D to minimum width
        PW0D = PWM_MIN_WIDTH;
        break;
    case PULSE_WIDTH_PERIODIC_START:  // PERIODIC mode reference value set to 1000W HCW**
        PW0D = 280;
        break;
    case PULSE_WIDTH_NO_CHANGE:  // **Keep PW0D unchanged**
    default:
        break;
  }
  
  // Enable PWM
  PW0M = mskPW0EN | PW0_DIV1 | PW0_HOSC | PW0_INVERS | mskPW0PO;
  
  // FW starts PWM for the first charge
  PW0M1 |= mskPGOUT;
  while((PW0M1&mskPGOUT) != 0); // Wait pulse end
  
  PW0M &= ~mskPW0EN;            // Disable PWM
  CM0M |= mskCM0SF;             // Enable CM0 pulse trigger
  PW0M |= mskPW0EN;             // Enable PWM
  
//  // Start a 16ms countdown
//  cntdown_timer_start(CNTDOWN_TIMER_POT_HEATING_CURRENT_DELAY , POT_HEATING_CURRENT_DELAY_MS); 
  
  reset_power_read_data();
  f_heating_initialized = 1;       // Mark heating as initialized
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#define SLOWDOWN_PWM_WIDTH  30    // 750ns / 31.25ns = 24 clks
#define SLOWDOWN_PWM_PERIOD 800   // 18.75us / 31.25ns = 600 clks

void IGBT_C_slowdown(void) {
    // Patch: Protect for PWM accidentally switch mode
    PW0M &= ~mskPW0EN;    // Disable PWM
    CM0M &= ~mskCM0SF;    // Disable CM0 pulse trigger
  
    PW0Y = SLOWDOWN_PWM_PERIOD;   // **設定 PWM 週期**
    PW0D = SLOWDOWN_PWM_WIDTH;    // **設定 PWM 脈衝寬度**

    // **開啟 PWM**
    PW0M = mskPW0EN | PW0_DIV1 | PW0_HOSC | PW0_INVERS | mskPWM0OUT;
}


// === End of power.c ===

// === Start of PWM.c ===
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



// === End of PWM.c ===

// === Start of system.c ===
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
#include "system.h"
#include "temperature.h"
#include <math.h>
#include "comparator.h"
#include "PWM.h"
#include "config.h"
#include "power.h"
#include "communication.h"

/*_____ D E F I N I T I O N S ______________________________________________*/
#define SYSTEM_TICKS_PER_1MS    8     // 125us*8 = 1ms
#define SYSTEM_1MS_PER_SECOND   1000  // 1ms*1000 = 1s

#define SHUTDOWN_DURATION_SECONDS 60 // 60秒
#define SHUTDOWN_TEMP_THRESHOLD 50 // 50度C

/*_____ D E C L A R A T I O N S ____________________________________________*/
static uint16_t cntdown_timers[MAX_CNTDOWN_TIMERS]; // 計時器數組
volatile bit f_125us = 0;
bit exit_shutdown_flag = 0;
bit f_shutdown_in_progress = 0;  // 關機進行標誌
bit f_pot_detected = 0;
bit f_pot_analyzed = 0;
uint16_t recorded_1000W_PW0D = 0;

uint32_t power_setting = 0;
uint32_t target_power = 0;            // 目標功率

volatile uint8_t system_ticks = 0;    // 系統計數 (125 μs 為單位)
uint16_t system_time_1ms = 0;         // 系統時間（1 ms 為單位）
uint16_t system_time_1s = 0;          // 系統時間（1 秒為單位

uint8_t measure_per_AC_cycle = MEASUREMENTS_PER_60HZ;

ErrorFlags error_flags;

SystemState system_state = STANDBY;  // 系統初始狀態為待機, should not switch state in ISR, it may switch back in main loop. 
  
  #if TUNE_MODE == 1
//  uint16_t xdata tune_cnt = 0;
  uint32_t xdata tune_record1 = 0;
  uint32_t xdata tune_record2 = 0;
  #endif

/*_____ M A C R O S ________________________________________________________*/

/*_____ F U N C T I O N S __________________________________________________*/
void Pot_Detection(void);
void Pot_Detection_In_Heating(void);
void shutdown_process(void);
void finalize_shutdown(void);

static bit shutdown_triggered = 0;           // 標記是否已觸發關機邏輯
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void SystemCLK_Init(void)
{
  // 32MHz /2 = 16MHz
  CLKSEL = SYS_CLK_DIV2;
  CLKCMD = 0x69;
  
  // ILRC Calibration 
  CLKCAL |= 0x80;
  FRQCMD = 0x4B;
  while((CLKCAL&0x80) != 0);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Shutdown_Task(void)
{    
//  static uint32_t shutdown_start_time = 0;        // 關機開始時間（以秒為單位）
//  uint8_t elapsed_time;

//  // 檢查 system_state 是否為 SHUTTING_DOWN
//  if (system_state != SHUTTING_DOWN) {
//      return;  // 如果不在 SHUTTING_DOWN 狀態，直接退出
//  }

//  // 初始化關機邏輯
//  if (!shutdown_triggered) {
//      shutdown_start_time = system_time_1s;       // 記錄關機開始時間
//      shutdown_process();                        // 執行一次性關機邏輯
//      shutdown_triggered = 1;
//  }

//  // 檢查是否允許跳脫關機 !!
//  if (exit_shutdown_flag) {
//      exit_shutdown_flag = 0;  // 清除 CMD 標誌      
//      shutdown_triggered = 0;       // 重置關機邏輯
//      shutdown_start_time = 0;      // 重置開始時間
//      f_shutdown_in_progress = 0;    // 結束關機
//      return;                      // 跳脫關機行為
//  }

//  // 計算已過時間
//  elapsed_time = system_time_1s - shutdown_start_time;

//  // 判斷是否完成關機條件
//  if (elapsed_time >= SHUTDOWN_DURATION_SECONDS &&
//      IGBT_TEMP_C < SHUTDOWN_TEMP_THRESHOLD &&
//      TOP_TEMP_C < SHUTDOWN_TEMP_THRESHOLD) {
//      finalize_shutdown();             // 完成關機操作
//      system_state = STANDBY;          // 切換系統狀態為 STANDBY
//      shutdown_triggered = 0;         // 重置關機邏輯
//      f_shutdown_in_progress = 0;        // 清除關機標誌
//  }
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// 初始化倒數計時模組
void CNTdown_Timer_Init() 
{
  uint8_t id;
    for (id = 0; id < MAX_CNTDOWN_TIMERS; id++) 
    {
        cntdown_timers[id] = 0; // 初始化所有計時器為 0
    }
}

// 開始倒數計時
void cntdown_timer_start(uint8_t timer_id, uint16_t duration) {
        cntdown_timers[timer_id] = duration; // 設置計時器初始值
}

// 檢查倒數計時器是否已到時間
uint8_t cntdown_timer_expired(uint8_t timer_id) {
    return (cntdown_timers[timer_id] == 0) ? 1 : 0;
}

// 更新所有倒數計時器（每 1ms 調用一次）
void cntdown_timer_update() {
  uint8_t i;
    for (i = 0; i < MAX_CNTDOWN_TIMERS; i++) {
        if (cntdown_timers[i] > 0) {
            cntdown_timers[i]--; // 遞減計時器
        }
    }
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Update_System_Time() {
    static uint8_t last_1ms_ticks = 0;    // 上次更新 1ms 的 ticks 值
    static uint16_t last_1s_time_1ms = 0;  // 上次 1 秒更新的 1 ms 記錄值

    // 利用 system_ticks 減去已記錄值來計算 1 ms
    if ((system_ticks - last_1ms_ticks) >= SYSTEM_TICKS_PER_1MS) {
        system_time_1ms++;                   // 增加 1ms 計數
        last_1ms_ticks = system_ticks; // 更新已記錄的 ticks

        // 更新所有倒數計時器
        cntdown_timer_update(); 
    }

    // 利用 system_time_1ms 減去已記錄值來計算 1 秒
    if ((system_time_1ms - last_1s_time_1ms) >= SYSTEM_1MS_PER_SECOND) {
        system_time_1s++;                     // 增加 1 秒計數
        last_1s_time_1ms = system_time_1ms; // 更新已記錄的 1ms 時間
    }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#define VOLTAGE_THRESHOLD 220          // 電壓門檻值 (220V)

#define HIGH_POWER_LEVEL 2000000       // 高功率標準 (2000K mW)
#define PWM_ADJUST_QUICK_CHANGE 3      // 電壓快速變化時減少的 PWM 寬度

uint32_t current_power = 0;           // 目前功率 mW
uint16_t target_current = 0;          // 目標電流 mA

uint16_t current_RMS_mA = 0;
uint16_t voltage_RMS_V = 0;

// 增加 PWM 寬度
void Increase_PWM_Width(uint8_t val) {
//    // 保存 CM2SF 狀態
//    uint8_t cm2sf_status = CM2M & mskCM2SF;

//    // 關閉 CM2SF 功能
//    CM2M &= ~mskCM2SF;
 
    // 增加 PWM 寬度
    if ((PW0D + val) <= PWM_MAX_WIDTH) { // 確保不超過最大值
        PW0D += val;
    } else {
        PW0D = PWM_MAX_WIDTH; // 避免超過最大寬度
    }
    
//    // 恢復 CM2SF 狀態
//    CM2M |= cm2sf_status;
}

// 減少 PWM 寬度
void Decrease_PWM_Width(uint8_t val) {
//    // 保存 CM2SF 狀態
//    uint8_t cm2sf_status = CM2M & mskCM2SF;

//    // 關閉 CM2SF 功能
//    CM2M &= ~mskCM2SF;

  // 減少 PWM 寬度
  if ((PW0D >= (val + PWM_MIN_WIDTH))) { // 確保不小於最小值
      PW0D -= val;
  } else {
      PW0D = PWM_MIN_WIDTH; // 避免低於最小寬度
  }
  
//    // 恢復 CM2SF 狀態
//    CM2M |= cm2sf_status;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// 功率控制函式
void Power_Control(void)
{
    // 檢查 5ms 倒數計時器是否已到
    if (!cntdown_timer_expired(CNTDOWN_TIMER_POWER_CONTROL)) {
        return; // 若未到 5ms，直接返回
    }
    // 重啟倒數計時器
    cntdown_timer_start(CNTDOWN_TIMER_POWER_CONTROL, 5);
  
    // If heating is not initialized, handle according to system_state
    if (!f_heating_initialized) {
        if (system_state == HEATING) {
            init_heating(HEATING_SYNC_AC, PULSE_WIDTH_MIN);
        }
        
        return;  // Exit if heating is not initialized (PERIODIC_HEATING case)
    }
    
    // 當功率大於高功率標準時處理
    if (current_power > HIGH_POWER_LEVEL) {
        if (error_flags.f.Voltage_quick_change) {
            // 電壓快速變化，減少 PWM 寬度
            Decrease_PWM_Width(PWM_ADJUST_QUICK_CHANGE);
            return; // 直接返回，避免進一步處理
        }
    }
    
//    // 判斷控制模式
//    if (voltage_RMS_V >= VOLTAGE_THRESHOLD) {
//        // 恆電流控制模式
//        target_current = target_power / VOLTAGE_THRESHOLD;
//    } else {
//        // 恆功率控制模式
//        target_current = target_power / voltage_RMS_V;
//    }
    
    #if TUNE_MODE == 1
//    tune_cnt++;
//    if(current_power >= tune_record)
//    {
//      tune_record = current_power;
//    }
//    
//    if(tune_cnt >= 600)
//    {
//      PW0M = 0;
//      _nop_();
//      //while(1);
//    }
    #endif
    
    // Patch for PW0D shrink to 0 by CM2SF
    if(PW0D < PWM_MIN_WIDTH) 
    {
      PW0M &= ~mskPW0EN;
      PW0D = PWM_MIN_WIDTH;
      PW0M |= mskPW0EN;
    }
    
//    // Compare target current with actual measured current
//    if (target_current > current_RMS_mA) {
//      Increase_PWM_Width(1); 
//    } else {
//      Decrease_PWM_Width(error_flags.f.Current_quick_large ? PWM_ADJUST_QUICK_CHANGE : 1);
//    }
    
    // Compare target power with actual measured current_power
    if (target_power > current_power) {
      Increase_PWM_Width(1); 
    } else {
      Decrease_PWM_Width(error_flags.f.Current_quick_large ? PWM_ADJUST_QUICK_CHANGE : 1);
    }
    
    #if TUNE_MODE == 1
//    if(tune_cnt >= 600)
//    {
//      while(1);
//    }
    #endif
    
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#define POT_CONFIRM_INTERVAL  2   // 鍋具確認倒數計時值 (2ms)
#define POT_CHECK_INTERVAL    500 // 鍋具檢測間隔倒數計時值 (500ms)
#define POT_PULSE_THRESHOLD   30  // 鍋具不存在的脈衝數門檻

PotDetectionState pot_detection_state = POT_IDLE;
volatile uint8_t pot_pulse_cnt = 0;   // 鍋具脈衝計數器

void Pot_Detection() {  
  
  if (!cntdown_timer_expired(CNTDOWN_TIMER_POT_DETECT)) {
    return;  // 檢鍋間隔時間未到，直接返回
  }
  
  f_CM3_AC_sync = 0; // Clear AC sync flag before waiting
  while (!f_CM3_AC_sync); // Block execution until AC low is detected
  
  
  switch (pot_detection_state) {
    case POT_IDLE:
      // **開始檢鍋**
      // 啟動鍋具確認倒數計時器
      cntdown_timer_start(CNTDOWN_TIMER_POT_DETECT, POT_CONFIRM_INTERVAL);
      
      pot_pulse_cnt = 0;  // 清零脈衝計數器
      pot_detection_state = POT_CHECKING;
      
      // Patch: Protect for PWM accidentally switch mode
      PW0M &= ~mskPW0EN;    // Disable PWM
      CM0M &= ~mskCM0SF;    // Disable CM0 pulse trigger
      PW0D = POT_DETECT_PULSE_TIME;    // 設定 PW0D 數值為檢鍋用值 (6us)
      PW0Y = PWM_MAX_WIDTH;             // 設定 PW0Y 為最大值 patch, 防止不小心反向
      // **觸發檢鍋**
      PW0M = mskPW0EN | PW0_DIV1 | PW0_HOSC | PW0_INVERS | mskPW0PO; // 設定 PWM0 為脈衝模式
      PW0M1 |= (mskPGOUT);             // 觸發檢鍋脈衝
      CLEAR_CM0_IRQ_FLAG;
      CM0_IRQ_ENABLE;
      break;
      
    case POT_CHECKING:
      // **等待檢鍋完成**
      if (!cntdown_timer_expired(CNTDOWN_TIMER_POT_DETECT)) {
        return;
      }
      // 鍋具確認完成，結束檢鍋邏輯
      CM0_IRQ_DISABLE;
      // 重啟檢鍋間隔計時器
      cntdown_timer_start(CNTDOWN_TIMER_POT_DETECT, POT_CHECK_INTERVAL);
        
      // **如果鍋具存在，轉入 `POT_ANALYZING`**
      if (pot_pulse_cnt < POT_PULSE_THRESHOLD) {
        f_pot_detected = 1;
        
        //pot_detection_state = POT_ANALYZING;
        //f_pot_analyzed = 0;
        //Pot_Analyze();
      } else {  // **鍋具不存在，保持在 `POT_IDLE`，下次重新檢測**
        //pot_detection_state = POT_IDLE;
        f_pot_detected = 0;
        error_flags.f.Pot_missing = 1;  // Raise missing pot flag
      }
      
      pot_detection_state = POT_IDLE;
      break;
      
    case POT_ANALYZING:
      Pot_Analyze();

      // **若 Pot_Analyze完成
      if (f_pot_analyzed == 1) {
        pot_detection_state = POT_IDLE;
      }
      break;
  }

}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 * @brief Checks pot presence during heating using current measurement.
 *
 * Detection is disabled for a short delay after heating starts,
 * controlled by @ref CNTDOWN_TIMER_POT_HEATING_CURRENT_DELAY.
 *
 * Sets @ref error_flags.f.Pot_missing if current is below 
 * @ref POT_PRESENT_CURRENT_MIN_mA. Does not clear the flag here.
 * The flag will be cleared in @ref Error_Process when system enters ERROR state.
 */

#define POT_PRESENT_CURRENT_MIN_mA  600 // I_RMS mA 

void Pot_Detection_In_Heating(void) 
{
  if (!f_heating_initialized)
  {
    return;
  }
  
//  // Skip detection if delay timer has not yet expired
//  if (!cntdown_timer_expired(CNTDOWN_TIMER_POT_HEATING_CURRENT_DELAY)) {
//    return;
//  }
  
  if (f_power_updated)
  {
    if (current_RMS_mA < POT_PRESENT_CURRENT_MIN_mA)
    {
      error_flags.f.Pot_missing = 1;  // Pot not detected due to low current
      // Simply stop the heating logic
      P01 = 1;  //PWM Pin
      PW0M = 0;
    }
  }
  
//   Old approach   
    // 僅在加熱狀態下執行檢鍋任務
//    if (system_state != HEATING) {
//        return; // 非 HEATING 狀態時直接返回
//    }

//    if (current_IIR_new < POT_CURRENT_THRESHOLD) {
//        // 電流低於門檻
//        if (!f_current_pot_checking) {
//            // 若未開始倒數，啟動倒數
//            cntdown_timer_start(CNTDOWN_TIMER_POT_CHECK, POT_CHECK_DELAY_MS);
//            f_current_pot_checking = 1; // 設置為正在倒數中
//        } else if (cntdown_timer_expired(CNTDOWN_TIMER_POT_CHECK)) {
//            // 若倒數完成，判定鍋具不存在
//            f_current_pot_checking = 0; // 清除倒數旗標
//            f_pot_detected = 0;        // 鍋具不存在
//            system_state = STANDBY;    // 切換系統狀態為待機
//        }
//    } else {
//        // 電流恢復正常
//        f_current_pot_checking = 0; // 清除倒數旗標
//        cntdown_timer_start(CNTDOWN_TIMER_POT_CHECK, 0); // 停止計時器
//    }

}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#define POT_ANALYZE_POWER   1000000   // 1000 000mW
#define DEFAULT_1000W_PW0D_VAL  320   // PWM 寬度 320cnt @32MHz = 10us
#define POWER_STABILITY_THRESHOLD  50000  // 50 000mW  
#define POWER_STABLE_TIME       1000   // 1000ms
#define POWER_SAMPLE_INTERVAL   100    // 100ms  

PotAnalyzeState pot_analyze_state = PWR_UP;

void Pot_Analyze(void) {
    static uint8_t xdata record_count = 0;
    static uint16_t xdata PW0D_val[4];  // 記錄 4 次 PWM 寬度
    uint16_t sum;
    uint8_t i;
    
    switch (pot_analyze_state) {
      case PWR_UP:
        // **第一步：開始加熱並進入穩定等待狀態**
        target_power = POT_ANALYZE_POWER;
        record_count = 0;  // 確保計數器歸零
        system_state = HEATING;

        // **啟動 1 秒倒數計時**
        cntdown_timer_start(CNTDOWN_TIMER_POT_DETECT, POWER_STABLE_TIME);
        pot_analyze_state = WAIT_STABILIZATION;
        break;

      case WAIT_STABILIZATION:        
        // **等待 1 秒穩定**
        if (!cntdown_timer_expired(CNTDOWN_TIMER_POT_DETECT)) {
            return; // 等待時間未到，保持當前狀態
        }

        // **1 秒結束後，開始記錄**
        cntdown_timer_start(CNTDOWN_TIMER_POT_DETECT, POWER_SAMPLE_INTERVAL); // 啟動 100ms 記錄計時
        pot_analyze_state = RECORDING;
        break;

      case RECORDING:
        // **等待 100ms 取樣間隔**
        if (!cntdown_timer_expired(CNTDOWN_TIMER_POT_DETECT)) {
            return;
        }

        // **重新啟動 100ms 記錄計時**
        cntdown_timer_start(CNTDOWN_TIMER_POT_DETECT, POWER_SAMPLE_INTERVAL);

        // **檢查功率是否穩定**
        if((current_power >= (POT_ANALYZE_POWER-POWER_STABILITY_THRESHOLD)) &&  \
           (current_power <= (POT_ANALYZE_POWER+POWER_STABILITY_THRESHOLD))     )
        { PW0D_val[record_count] = PW0D; }
        else
        { PW0D_val[record_count] = DEFAULT_1000W_PW0D_VAL; }
        record_count++;
        
        // **當 4 次記錄完成後，計算平均**
        if (record_count >= 4) {
          // Align with the AC zero-crossing to prevent surge protection 
          // from triggering due to rapid voltage rebound after pause_heating.
          f_CM3_AC_sync = 0;
          while(f_CM3_AC_sync == 0);
          // 停止加熱
          pause_heating();
          
          sum = 0;
          for (i = 0; i < 4; i++) {
              sum += PW0D_val[i];
          }
          recorded_1000W_PW0D = (sum>>2);  // 記錄平均值 = sum/4
          if(recorded_1000W_PW0D > 640)
          {
            recorded_1000W_PW0D  = 640; //HCW**
          }
          // **分析完成設置 `f_pot_detected = 1`**
          f_pot_detected = 1;
          f_pot_analyzed = 1;
          // **重置狀態，以便下次測量**
          pot_analyze_state = PWR_UP;
        }
        break;
    }
    
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#define AC_PERIOD_COUNT 4  // 取 4 次週期平均
#define DEBOUNCE_TICKS 2   // 軟體 debounce 時間 (2 * 125us = 250us)
#define PRE_CLEAR_DEBOUNCE_COUNT 3  // 清除中斷前需要 3 次連續確認 HIGH
#define PRE_CLEAR_DEBOUNCE_INTERVAL 2  // 每 1 個 `system_ticks` 確認一次 (250us)

uint8_t ac_low_periods[AC_PERIOD_COUNT] = {0};  // 記錄最近 4 次 AC 週期時間

uint8_t ac_half_low_ticks_avg;

void Measure_AC_Low_Time(void) {
    uint8_t i;
    uint8_t sum = 0;
    uint8_t start_ticks;
//  uint8_t low_count;

//    // **等待中斷發生**           // It’s not necessary because, after applying the Warmup_Delay and the 4ms debounce in the CM1_ISR, 
//    while (f_CM3_AC_sync == 0);   // we can ensure the CM3_last_sync_tick is only set on the AC’s rising edge.
  
    // 等待 AC 訊號穩定，收集 4 次數據
    for (i = 0; i < AC_PERIOD_COUNT; i++) {
      
//      // 軟體 debounce：確保 CM3 真的回到 LOW，f_CM3_AC_sync
//      low_count = 0;
//      while (low_count < PRE_CLEAR_DEBOUNCE_COUNT) {
//        debounce_start = system_ticks;
//        while ((system_ticks - debounce_start) < PRE_CLEAR_DEBOUNCE_INTERVAL); // 等待 `250us`
//        
//        if ((CMOUT & mskCM3OUT) == 0) {
//            low_count++;
//        } else {
//            low_count = 0;  // 若 `CM1` 變 LOW，則重新計算
//        }
//      }
      
      f_CM3_AC_sync = 0;  // **清除中斷標誌**

      // **等待中斷發生**
      while (f_CM3_AC_sync == 0);
      
      // **開始計時**
      start_ticks = system_ticks;

      // 軟體 debounce：等待至少 `DEBOUNCE_TICKS` 再開始偵測**
      while ((system_ticks - start_ticks) < DEBOUNCE_TICKS);
      
      // **等待 CM3 輸出轉為 LOW**
      while (CMOUT & mskCM3OUT);
      
      // **記錄時間**
      ac_low_periods[i]  = system_ticks - start_ticks;
      WDTR = 0x5A; // Clear watchdog
    }

    // 計算 4 次週期的平均值
    for (i = 0; i < AC_PERIOD_COUNT; i++) {
        sum += ac_low_periods[i];
    }
    ac_half_low_ticks_avg = sum / (AC_PERIOD_COUNT*2);  // 平均值 = sum / (AC_PERIOD_COUNT*2)
    
    if ((ac_half_low_ticks_avg>16) || (ac_half_low_ticks_avg<4))
    {ac_half_low_ticks_avg = 6;}
    
    ac_half_low_ticks_avg += 3;   // special modify for not symmetry divider circuit.
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#define AC_FREQUENCY_SAMPLES 4   // 取 4 次測量平均
#define AC_60HZ_THRESHOLD  73    // 60Hz 判斷門檻（73 system_ticks）

bit f_AC_50Hz = 0; // **AC 頻率標誌，1 = 50Hz，0 = 60Hz**
uint8_t xdata ac_ticks[AC_FREQUENCY_SAMPLES];

void Detect_AC_Frequency(void) {
    uint8_t i;
    uint16_t sum = 0;
    uint8_t start_tick, elapsed_ticks;

    f_CM3_AC_sync = 0;
  
    for (i = 0; i < AC_FREQUENCY_SAMPLES; i++) {
        // **等待 f_CM3_AC_sync 立起**
        while (!f_CM3_AC_sync);
        f_CM3_AC_sync = 0;  // **清除標誌**
        
        // **記錄開始時間**
        start_tick = system_ticks;

        // **等待下一次 f_CM3_AC_sync 立起**
        while (!f_CM3_AC_sync);
        f_CM3_AC_sync = 0;  // **清除標誌**
        
        // **計算經過的 ticks**
        elapsed_ticks = system_ticks - start_tick;
        ac_ticks[i] = elapsed_ticks;  // **存入 xdata 陣列**
        WDTR = 0x5A; // Clear watchdog
    }

    // **計算 4 次測量的平均值**
    for (i = 0; i < AC_FREQUENCY_SAMPLES; i++) {
        sum += ac_ticks[i];
    }
    sum /= AC_FREQUENCY_SAMPLES;

    // **判斷 AC 頻率 (50Hz / 60Hz)**
    f_AC_50Hz = (sum >= AC_60HZ_THRESHOLD) ? 1 : 0;
    
    measure_per_AC_cycle = (f_AC_50Hz) ? MEASUREMENTS_PER_50HZ : MEASUREMENTS_PER_60HZ;
    
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#define ERROR_RECOVERY_TIME_S  2  // Time required for error recovery (seconds)

ErrorFlags error_flags = {0};
volatile uint8_t Surge_Overvoltage_Flag = 0;
volatile uint8_t Surge_Overcurrent_Flag = 0;

void Error_Process(void)
{
  static uint8_t error_clear_time_1s  = 0; // Record the last time all errors were cleared
    
  // If system is not in ERROR state, check if it needs to enter ERROR
  if (system_state != ERROR) {
    if (Surge_Overvoltage_Flag || Surge_Overvoltage_Flag || error_flags.all_flags) {
      system_state = ERROR;
      
      error_clear_time_1s = system_time_1s;  // Record the current time
      
      // Stop_heating again
      stop_heating();
      
      // pot_detection & pot_analyze ini
      pot_detection_state = POT_IDLE;
      //pot_analyze_state = PWR_UP; //HCW**Cancel the pot analysis process.
      
      // Set I2C status code based on error type
      if (Surge_Overvoltage_Flag)
        i2c_status_code = I2C_STATUS_OVERVOLTAGE;
      else if (Surge_Overcurrent_Flag)
        i2c_status_code = I2C_STATUS_OVERCURRENT;
      else
        i2c_status_code = I2C_STATUS_OTHER_ERROR;
      
      
      P10 = ~P10 ;//HCW**
    }
    return;
  }

  // Check if Surge Overvoltage has recovered
  if (Surge_Overvoltage_Flag) {
      if (CMOUT & mskCM1OUT) {  // If CM1OUT returns to 1, voltage is back to normal
          Surge_Overvoltage_Flag = 0;
      }
  }
  
  // Check if Surge Overcurrent has recovered
  if (Surge_Overcurrent_Flag) {
      if (CMOUT & mskCM4OUT) {  // If CM4OUT returns to 1, current is back to normal
          Surge_Overcurrent_Flag = 0;
      }
  }
  
  // Clear Pot_missing flag after entering ERROR state
  if (error_flags.f.Pot_missing) {
    error_flags.f.Pot_missing = 0;
  }

  // If any error flags are still active, update the error_clear_time_1s and return
  if (Surge_Overvoltage_Flag || Surge_Overcurrent_Flag || error_flags.all_flags) {
      error_clear_time_1s = system_time_1s;  // Record the current time
      return;
  }

  // All errors cleared, check if ERROR_RECOVERY_TIME_S seconds have passed
  if ((uint8_t)(system_time_1s - error_clear_time_1s) >= ERROR_RECOVERY_TIME_S) {
      system_state = STANDBY; // Maintain ERROR_RECOVERY_TIME_S seconds without errors before returning to STANDBY
      i2c_status_code = I2C_STATUS_NORMAL;  // clear status to normal
  }
  
}




// === End of system.c ===

// === Start of temperature.c ===
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
#include "temperature.h"
#include "config.h"
#include "ADC.h"
#include "system.h"

/*_____ D E F I N I T I O N S ______________________________________________*/
#define TEMP_ACCUMULATE_COUNT 8

/*_____ D E C L A R A T I O N S ____________________________________________*/
uint16_t IGBT_TEMP_code = 0;        // 目前IGBT溫度 AD_code
uint16_t TOP_TEMP_code = 0;         // 目前表面溫度 AD_code
int IGBT_TEMP_C = 0;                // 目前IGBT溫度 度C
int TOP_TEMP_C = 0;                 // 目前表面溫度 AD_code
static bit f_temp_updated = 0;      // 溫度變數更新旗標

/**
* The NTC table has 129 interpolation points.
* Unit: 1°C
* Description: 	NTC with pull up resistor
*				NTC resistance at 25°C 	  :100K
*				Pullup-resistance 	      :20K
*				BETA of NTC 			        :3950
*				ADC Resolution 			      :12bit
* Table Generator: https://www.sebulli.com/ntc/
*/
const int code IGBT_NTC_table[129] = {
	362, 308, 254, 226, 208, 195, 185, 176, 169, 
  163, 158, 153, 148, 144, 141, 137, 134, 131, 
  129, 126, 124, 121, 119, 117, 115, 113, 111, 
  110, 108, 106, 105, 103, 102, 100, 99, 97, 
  96, 95, 93, 92, 91, 90, 88, 87, 86, 85, 84, 
  83, 82, 81, 80, 79, 78, 77, 76, 75, 74, 73, 
  72, 71, 70, 69, 68, 67, 66, 65, 64, 64, 63, 
  62, 61, 60, 59, 58, 57, 56, 56, 55, 54, 53, 
  52, 51, 50, 49, 48, 47, 47, 46, 45, 44, 43, 
  42, 41, 40, 39, 38, 37, 36, 35, 34, 33, 32, 
  31, 29, 28, 27, 26, 25, 23, 22, 21, 19, 18, 
  16, 14, 13, 11, 9, 7, 5, 2, -1, -4, -7, -11, 
  -16, -23, -34, -45
};

/**
* The NTC table has 129 interpolation points.
* Unit: 1°C
* Description: 	NTC with pull down resistor
*				NTC resistance at 25°C 	  :100K
*				Pulldown-resistance 	    :3K
*				BETA of NTC 			        :3950
*				ADC Resolution 			      :12bit
* Table Generator: https://www.sebulli.com/ntc/
*/
const int code TOP_NTC_table[129] = {
	-16, -2, 12, 20, 27, 32, 37, 41, 44, 47, 
  50, 53, 56, 58, 61, 63, 65, 67, 69, 71, 73, 
  74, 76, 78, 79, 81, 82, 84, 86, 87, 88, 90, 
  91, 93, 94, 95, 97, 98, 99, 101, 102, 103, 
  105, 106, 107, 108, 110, 111, 112, 113, 115, 
  116, 117, 118, 120, 121, 122, 123, 125, 126, 
  127, 128, 130, 131, 132, 134, 135, 136, 138, 
  139, 140, 142, 143, 144, 146, 147, 149, 150, 
  152, 153, 155, 156, 158, 160, 161, 163, 165, 
  166, 168, 170, 172, 174, 176, 178, 180, 182, 
  184, 186, 188, 191, 193, 196, 199, 201, 204, 
  207, 210, 214, 217, 221, 225, 229, 234, 239, 
  244, 249, 256, 262, 270, 279, 289, 300, 314, 
  332, 354, 385, 434, 539, 644
};

/*_____ M A C R O S ________________________________________________________*/

/*_____ F U N C T I O N S __________________________________________________*/
void Temp_Measure(void)
{
  uint16_t temp_raw;
  static uint16_t IGBT_TEMP_sum = 0;      // 累加 IGBT 溫度的變數
  static uint16_t TOP_TEMP_sum = 0;       // 累加表面溫度的變數
  static uint8_t Temp_Measure_cnt = 0;    // 量測次數計數器  
  
  // 呼叫通用 ADC 量測函式，量測 IGBT 溫度
  ADC_measure_4_avg(IGBT_TEMP_ADC_CHANNEL, &temp_raw);
  IGBT_TEMP_sum += temp_raw;

  // 呼叫通用 ADC 量測函式，量測表面溫度
  ADC_measure_4_avg(TOP_TEMP_ADC_CHANNEL, &temp_raw);
  
  #if ICE_DEBUG_MODE == 1
  temp_raw = 124;
  #endif
  TOP_TEMP_sum += temp_raw;

  // 增加量測次數
  Temp_Measure_cnt++;

  // 每 8 次執行一次平均計算並更新變數
  if (Temp_Measure_cnt >= TEMP_ACCUMULATE_COUNT) {
    IGBT_TEMP_code = IGBT_TEMP_sum>>3;    // 更新IGBT溫度 sum/8
    TOP_TEMP_code = TOP_TEMP_sum>>3;      // 更新表面溫度 sum/8

    // 設置旗標，表示溫度變數已更新
    f_temp_updated = 1;

    // 重置累加器與計數器
    IGBT_TEMP_sum = 0;
    TOP_TEMP_sum = 0;
    Temp_Measure_cnt = 0;
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// 定義 IGBT 溫度上下限
#define IGBT_TEMP_UPPER_LIMIT  60   // IGBT 溫度上限 (攝氏度)60
#define IGBT_TEMP_LOWER_LIMIT -20   // IGBT 溫度下限 (攝氏度)
#define IGBT_TEMP_RECOVERY     40   // IGBT Overheat release temperature 40

// 定義 TOP 溫度上下限
#define TOP_TEMP_UPPER_LIMIT  180   // TOP 溫度上限 (單位：適當的測量單位，例如攝氏度)
#define TOP_TEMP_LOWER_LIMIT  -20   // TOP 溫度下限 (單位：適當的測量單位，例如攝氏度)
#define TOP_TEMP_RECOVERY     100   // TOP Overheat release temperature


void Temp_Process(void)
{
  int p1,p2;
  
  // 檢查是否有更新的溫度數據
  if (f_temp_updated) {
    f_temp_updated = 0;  // 清除溫度更新旗標
    
    //IGBT_TEMP_code += TMP_CAL_Offset;
  
    // IGBT temperature process
    /* Estimate the interpolating point before and after the ADC value. */
    p1 = IGBT_NTC_table[ (IGBT_TEMP_code >> 5)  ];
    p2 = IGBT_NTC_table[ (IGBT_TEMP_code >> 5)+1];
    
    /* Interpolate between both points. */
    IGBT_TEMP_C = p1 - (((p1-p2)*(IGBT_TEMP_code & 0x001F)) >> 5);  // = p1 + ( (p2-p1) * (IGBT_TEMP_code & 0x001F) ) / 32
    
    // 檢查溫度是否異常 IGBT_Temp_Process
    if (IGBT_TEMP_C > IGBT_TEMP_UPPER_LIMIT) {
      error_flags.f.IGBT_overheat = 1;
    } else if (IGBT_TEMP_C < IGBT_TEMP_RECOVERY) {
      error_flags.f.IGBT_overheat = 0;
    }

    if (IGBT_TEMP_C < IGBT_TEMP_LOWER_LIMIT) {
      error_flags.f.IGBT_sensor_fault = 1;
    } else {
      error_flags.f.IGBT_sensor_fault = 0;
    }
    
    // TOP temperature process
    /* Estimate the interpolating point before and after the ADC value. */
    p1 = TOP_NTC_table[ (TOP_TEMP_code >> 5)  ];
    p2 = TOP_NTC_table[ (TOP_TEMP_code >> 5)+1];
    
    /* Interpolate between both points. */
    TOP_TEMP_C = p1 + (((p2-p1)*(TOP_TEMP_code & 0x001F)) >> 5);  // = p1 + ( (p2-p1) * (IGBT_TEMP_code & 0x001F) ) / 32
    
    // 檢查溫度是否異常 TOP_Temp_Process
    if (TOP_TEMP_C > TOP_TEMP_UPPER_LIMIT) {
      error_flags.f.TOP_overheat = 1;
    } else if (TOP_TEMP_C < TOP_TEMP_RECOVERY) {
      error_flags.f.TOP_overheat = 0;
    }

    if (TOP_TEMP_C < TOP_TEMP_LOWER_LIMIT) {
      error_flags.f.TOP_sensor_fault = 1;
    } else {
      error_flags.f.TOP_sensor_fault = 0;
    }
  }
}


// === End of temperature.c ===

// === Start of timer0.c ===
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
void Timer0_Init() {
  // 計算 Timer0 的初始值對應 32 MHz 系統時鐘
  #define TIMER0_RELOAD_VALUE (0xFFFF - 3999)

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
    system_ticks++;  // 增加 125 μs 計數
    f_125us = 1;  // 立起 125 μs 旗標
}
// === End of timer0.c ===

