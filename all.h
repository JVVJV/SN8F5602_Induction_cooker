// === Start of ADC.h ===
#ifndef __ADC_H__
#define __ADC_H__

/*_____ I N C L U D E S ____________________________________________________*/
#include <SN8F5602.h>


/*_____ D E C L A R A T I O N S ____________________________________________*/


/*_____ D E F I N I T I O N S ______________________________________________*/
// ADM
#define mskADENB        (1<<7)
#define mskADS          (1<<6)
#define mskEOC          (1<<5)
// ADR
#define ADC_CLK_HOSC    (0<<7)
#define ADC_CLK_LOSC    (1<<7)
#define mskGCHS         (1<<6)
// VREFH
#define VREF_INTERNAL   (0<<7)
#define VREF_EXTERNAL   (1<<7)
#define VREF_VDD        (4<<0)
#define VREF_2V5        (0<<0)
#define VREF_3V5        (1<<0)
#define VREF_4V5        (2<<0)
#define VREF_1V5        (3<<0)
// ADCAL
#define mskACS          (1<<7)
#define mskCALIVALENB   (1<<6)
#define SAMPLE_3T5      (0<<4)
#define SAMPLE_4T5      (1<<4)
#define SAMPLE_6T5      (2<<4)
#define SAMPLE_10T5     (3<<4)
#define ADC_CLK_DIV1    (0<<0)
#define ADC_CLK_DIV2    (1<<0)
#define ADC_CLK_DIV4    (2<<0)
#define ADC_CLK_DIV8    (3<<0)
#define ADC_CLK_DIV16   (4<<0)
#define ADC_CLK_DIV32   (5<<0)
#define ADC_CLK_DIV64   (6<<0)
#define ADC_CLK_DIV128  (7<<0)




/*_____ M A C R O S ________________________________________________________*/
#define START_ADC_CONVERSION  (ADM |= mskADS)
#define IS_ADC_FINISH   (ADM&mskEOC)
#define CLEAR_EOC       (ADM &= ~mskEOC)
#define GET_ADC_RESULT  ((ADB<<4)|(ADR&0x0F))

/*_____ F U N C T I O N S __________________________________________________*/
void ADC_Init(void);
void ADC_measure_4_avg(uint8_t channel, uint16_t *result);

#endif  // __ADC_H__

// === End of ADC.h ===

// === Start of buzzer.h ===
#ifndef __BUZZER_H__
#define __BUZZER_H__

/*_____ I N C L U D E S ____________________________________________________*/
#include <SN8F5602.h>


/*_____ D E C L A R A T I O N S ____________________________________________*/


/*_____ D E F I N I T I O N S ______________________________________________*/
// PW0M
#define mskBZEN        (1<<7)
#define BZ_RATE_512    (0<<4)
#define BZ_RATE_1024   (1<<4)
#define BZ_RATE_2048   (2<<4)
#define BZ_RATE_4096   (3<<4)
#define BZ_RATE_8192   (4<<4)
#define BZ_RATE_16384  (5<<4)
#define BZ_RATE_32768  (6<<4)
#define BZ_RATE_65536  (7<<4)


/*_____ M A C R O S ________________________________________________________*/
#define BUZZER_ENABLE   BZM |= mskBZEN
#define BUZZER_DISABLE   BZM &= (~mskBZEN)

/*_____ F U N C T I O N S __________________________________________________*/
void Buzzer_Init(void);

#endif  // __PWM_H__

// === End of buzzer.h ===

// === Start of communication.h ===
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
#include <SN8F5602.h>


/*_____ D E F I N I T I O N S ______________________________________________*/
#define I2C_STATUS_NORMAL           0x70  // Device normal
#define I2C_STATUS_OVERVOLTAGE      0x71  // Surge: Over voltage
#define I2C_STATUS_OVERCURRENT      0x72  // Surge: Over current
#define I2C_STATUS_OTHER_ERROR      0x73  // Other error flag active
#define I2C_STATUS_IGBT_OVERHEAT    0x74  // 

/*_____ D E C L A R A T I O N S ____________________________________________*/
extern uint8_t i2c_status_code;  // default normal status

/*_____ M A C R O S ________________________________________________________*/

/*_____ F U N C T I O N S __________________________________________________*/
void I2C_Communication(void);



// === End of communication.h ===

// === Start of comparator.h ===
#ifndef __COMPARATOR_H__
#define __COMPARATOR_H__

/*_____ I N C L U D E S ____________________________________________________*/
#include <SN8F5602.h>


/*_____ D E F I N I T I O N S ______________________________________________*/
// CM0M
#define mskCM0EN        (1<<7)
#define mskCM0SF        (1<<4)

#define CM0_RISING_TRIGGER    (1<<3)
#define CM0_FALLING_TRIGGER   (0<<3)
#define CM0_CLK_HOSC      (0<<2)
#define CM0_CLK_FCPU      (1<<2)
// CMDB0
#define DELAY_0T       (0<<4)
#define DELAY_4T       (1<<4)
#define DELAY_8T       (2<<4)
#define DELAY_12T      (3<<4)
#define DELAY_16T      (4<<4)
#define DELAY_20T      (5<<4)
#define DELAY_24T      (6<<4)
#define DELAY_28T      (7<<4)
#define DELAY_32T      (8<<4)
#define DELAY_36T      (9<<4)
#define DELAY_40T      (10<<4)
#define DELAY_44T      (11<<4)
#define DELAY_48T      (12<<4)
#define DELAY_52T      (13<<4)
#define DELAY_56T      (14<<4)
#define DELAY_60T      (15<<4)

#define DEBOUNCE_0FCPU      (0<<0)
#define DEBOUNCE_2FCPU      (1<<0)
#define DEBOUNCE_4FCPU      (2<<0)
#define DEBOUNCE_6FCPU      (3<<0)
#define DEBOUNCE_8FCPU      (4<<0)
#define DEBOUNCE_10FCPU     (5<<0)
#define DEBOUNCE_12FCPU     (6<<0)
#define DEBOUNCE_14FCPU     (7<<0)
#define DEBOUNCE_16FCPU     (8<<0)
#define DEBOUNCE_18FCPU     (9<<0)
#define DEBOUNCE_20FCPU     (10<<0)
#define DEBOUNCE_22FCPU     (11<<0)
#define DEBOUNCE_24FCPU     (12<<0)
#define DEBOUNCE_26FCPU     (13<<0)
#define DEBOUNCE_28FCPU     (14<<0)
#define DEBOUNCE_30FCPU     (15<<0)

// CM1M
#define mskCM1EN        (1<<7)
#define mskCM1SF        (1<<4)

#define CM1_RISING_TRIGGER    (1<<3)
#define CM1_FALLING_TRIGGER   (0<<3)
#define CM1_CLK_HOSC          (0<<2)
#define CM1_CLK_FCPU          (1<<2)

// CM1REF
#define CM1REF_VDD      (0<<7)
#define CM1REF_INTREF   (1<<7)

// CMDB1
#define CM1_DEBOUNCE_0FCPU      (0<<0)
#define CM1_DEBOUNCE_2FCPU      (1<<0)
#define CM1_DEBOUNCE_4FCPU      (2<<0)
#define CM1_DEBOUNCE_6FCPU      (3<<0)
#define CM1_DEBOUNCE_8FCPU      (4<<0)
#define CM1_DEBOUNCE_10FCPU     (5<<0)
#define CM1_DEBOUNCE_12FCPU     (6<<0)
#define CM1_DEBOUNCE_14FCPU     (7<<0)
#define CM1_DEBOUNCE_16FCPU     (8<<0)
#define CM1_DEBOUNCE_18FCPU     (9<<0)
#define CM1_DEBOUNCE_20FCPU     (10<<0)
#define CM1_DEBOUNCE_22FCPU     (11<<0)
#define CM1_DEBOUNCE_24FCPU     (12<<0)
#define CM1_DEBOUNCE_26FCPU     (13<<0)
#define CM1_DEBOUNCE_28FCPU     (14<<0)
#define CM1_DEBOUNCE_30FCPU     (15<<0)

// CM2M
#define mskCM2EN    (1<<7)
#define mskCM2SF    (1<<4)

#define CM2_RISING_TRIGGER    (1<<3)
#define CM2_FALLING_TRIGGER   (0<<3)
// CMDB2

// CM2REF
#define CM2REF_VDD      (0<<7)
#define CM2REF_INTREF   (1<<7)


// CM3M
#define mskCM3EN    (1<<7)

#define CM3_RISING_TRIGGER    (1<<3)
#define CM3_FALLING_TRIGGER   (0<<3)
// CMDB3
#define CM3_DEBOUNCE_0FCPU      (0<<0)
#define CM3_DEBOUNCE_2FCPU      (1<<0)
#define CM3_DEBOUNCE_4FCPU      (2<<0)
#define CM3_DEBOUNCE_6FCPU      (3<<0)
#define CM3_DEBOUNCE_8FCPU      (4<<0)
#define CM3_DEBOUNCE_10FCPU     (5<<0)
#define CM3_DEBOUNCE_12FCPU     (6<<0)
#define CM3_DEBOUNCE_14FCPU     (7<<0)
#define CM3_DEBOUNCE_16FCPU     (8<<0)
#define CM3_DEBOUNCE_18FCPU     (9<<0)
#define CM3_DEBOUNCE_20FCPU     (10<<0)
#define CM3_DEBOUNCE_22FCPU     (11<<0)
#define CM3_DEBOUNCE_24FCPU     (12<<0)
#define CM3_DEBOUNCE_26FCPU     (13<<0)
#define CM3_DEBOUNCE_28FCPU     (14<<0)
#define CM3_DEBOUNCE_30FCPU     (15<<0)

// CM3REF
#define CM3REF_VDD      (0<<7)
#define CM3REF_INTREF   (1<<7)

// CM4M
#define mskCM4EN    (1<<7)
#define mskCM4SF    (1<<4)

#define CM4N_CM4N_PIN   (0<<1)
#define CM4N_CM0P       (1<<1)
#define CM4N_OPO        (2<<1)

#define CM4_RISING_TRIGGER    (1<<3)
#define CM4_FALLING_TRIGGER   (0<<3)

// CM4REF
#define CM4REF_VDD      (0<<7)
#define CM4REF_INTREF   (1<<7)


// CMOUT
#define mskCM4OUT     (1<<4)
#define mskCM3OUT     (1<<3)
#define mskCM2OUT     (1<<2)
#define mskCM1OUT     (1<<1)
#define mskCM0OUT     (1<<0)

// INTREF
#define mskINTREFEN   (1<<7)
#define INTREF2V5     (0<<0)
#define INTREF3V5     (1<<0)
#define INTREF4V5     (2<<0)
#define INTREF1V5     (3<<0)

// TCON
#define mskTF0      (1<<5)

// IEN2
#define mskECMP4    (1<<7)
#define mskECMP3    (1<<6)
#define mskECMP2    (1<<5)
#define mskECMP1    (1<<4)
#define mskECMP0    (1<<3)

// IRCON2
#define mskCM0F     (1<<3)

/*_____ D E C L A R A T I O N S ____________________________________________*/
extern volatile uint8_t ISR_f_CM3_AC_sync;
extern volatile uint8_t ISR_f_CM3_AC_Zero_sync;
extern volatile uint8_t CM3_AC_sync_cnt;
extern volatile uint8_t CM3_last_sync_tick;


/*_____ M A C R O S ________________________________________________________*/
#define CM0_IRQ_ENABLE  IEN2 |= mskECMP0
#define CM0_IRQ_DISABLE IEN2 &= (~mskECMP0)

#define CLEAR_CM0_IRQ_FLAG IRCON2 &= (~mskCM0F)

/*_____ F U N C T I O N S __________________________________________________*/
void Comparator_Init(void);
void Surge_Protection_Modify(void);

#endif  // __COMPARATOR_H__

// === End of comparator.h ===

// === Start of config.h ===
#ifndef __CONFIG_H__
#define __CONFIG_H__

/*_____ I N C L U D E S ____________________________________________________*/
#include <SN8F5602.h>


/*_____ D E C L A R A T I O N S ____________________________________________*/


/*_____ D E F I N I T I O N S ______________________________________________*/
#define ICE_DEBUG_MODE 1

#define TUNE_MODE     1

/*
//P00 =1;
//P00 =0;

//P10 = 1;
//P10 = 0;
*/

/*_____ M A C R O S ________________________________________________________*/

/*_____ F U N C T I O N S __________________________________________________*/
#define MEASUREMENTS_PER_60HZ 133     // 每 60 Hz 完整週期所需的量測次數 (base on 125us)
#define MEASUREMENTS_PER_50HZ 160     // 每 50 Hz 完整週期所需的量測次數 (base on 125us)

// AC Frequency Mode Control
#define AC_FREQ_MODE_AUTO         0
#define AC_FREQ_MODE_FORCE_50HZ   1
#define AC_FREQ_MODE_FORCE_60HZ   2

// Select desired AC frequency mode here:
#define AC_FREQ_MODE  AC_FREQ_MODE_AUTO


#define CURRENT_ADC_CHANNEL       19  // OPO  定義系統電流量測的 ADC 通道 
#define VOLTAGE_ADC_CHANNEL       7   // AIN7 定義電網電壓量測的 ADC 通道
#define IGBT_TEMP_ADC_CHANNEL     1   // AIN1
#define TOP_TEMP_ADC_CHANNEL      2   // AIN2

#define PERIODIC_TARGET_POWER     1000000 // 1000 000mW


#define POT_DETECT_PULSE_TIME     192         // 設定 PW0D 數值為檢鍋用值 (6us)

//#define PWM_MAX_WIDTH           960         // PWM 最大寬度   960cnt @32MHz = 30us
//#define PWM_MAX_WIDTH           896         // PWM 最大寬度   896cnt @32MHz = 28us
#define PWM_MAX_WIDTH           768         // PWM 最大寬度   768cnt @32MHz = 24us
//#define PWM_MAX_WIDTH           704         // PWM 最大寬度   704cnt @32MHz = 22us
//#define PWM_MAX_WIDTH           640         // PWM 最大寬度   640cnt @32MHz = 20us
//#define PWM_MAX_WIDTH           512         // PWM 最大寬度   512cnt @32MHz = 16us
//#define PWM_MAX_WIDTH           417         // PWM 最大寬度   417cnt @32MHz = 13us
//#define PWM_MAX_WIDTH           320         // PWM 最大寬度   320cnt @32MHz = 10us //HCW**
//#define PWM_MAX_WIDTH           256         // PWM 最大寬度   250cnt @32MHz = 8us


#define PWM_MIN_WIDTH            224         // PWM 最小寬度   192cnt @32MHz = 7us
                                             // This value should not be too small, as a smaller value 
                                             // may increase the proportion of hard-switching operation, 
                                             // leading to IGBT overheating.

#define	I2C_SLAVE_ADDRESS					0x55
#define I2C_INTERVAL   43    // **I2C 操作間隔 43ms**

#endif  // __CONFIG_H__
// === End of config.h ===

// === Start of gpio.h ===
#ifndef __GPIO_H__
#define __GPIO_H__

/*_____ I N C L U D E S ____________________________________________________*/
#include <SN8F5602.h>


/*_____ D E F I N I T I O N S ______________________________________________*/

/*_____ D E C L A R A T I O N S ____________________________________________*/

/*_____ M A C R O S ________________________________________________________*/

/*_____ F U N C T I O N S __________________________________________________*/
void GPIO_Init(void);


#endif  // __GPIO_H__

// === End of gpio.h ===

// === Start of I2C.h ===
#ifndef __I2C_H__
#define __I2C_H__

/*_____ I N C L U D E S ____________________________________________________*/
#include <SN8F5602.h>

/*_____ D E F I N I T I O N S ______________________________________________*/
#define SLAVE_SEL                 0
#define MASTER_SEL                1

#define I2C_R                     1
#define I2C_W                     0

#define	mskI2CCON_I2C_DIV_40			(0x00)
#define	mskI2CCON_I2C_DIV_80			(0x01)
#define	mskI2CCON_I2C_DIV_160			(0x02)
#define	mskI2CCON_I2C_DIV_320			(0x03)

#define	mskI2CCON_I2C_AA								(0x01U<<2)
#define	mskI2CCON_I2C_FLAG							(0x01U<<3)
#define	mskI2CCON_I2C_STO								(0x01U<<4)
#define	mskI2CCON_I2C_STA								(0x01U<<5)
#define	mskI2CCON_I2C_ENABLE						(0x01U<<6)
#define	mskI2CCON_I2C_CLOCKDIV_T1OV			(0x01U<<7)

#define	mskI2CADR_I2C_GENERALCALL				(0x01U<<0)

#define mskI2CDAT_I2C_MODE_TX						(0x00U<<0)
#define mskI2CDAT_I2C_MODE_RX						(0x01U<<0)

#define	mskIEN1_INT_I2C									(0x01U<<0)


#define	mskI2CMX    (1<<3)


#define I2C_BUFFER_SIZE  3  // **最大緩衝區長度**

/*_____ D E C L A R A T I O N S ____________________________________________*/
extern volatile bit f_i2c_power_received;   // **功率接收完成標誌**

/*_____ M A C R O S ________________________________________________________*/
#define I2C_PIN_SET_INPUT_0     P0M &= ~((0x01U<<6)|(0x01U<<7));  //SCL = P06,SDA = P07.
#define I2C_PIN_SET_INPUT_1     P1M &= ~((0x01U<<3)|(0x01U<<4));	//SCL = P13,SDA = P14.

/*_____ F U N C T I O N S __________________________________________________*/
void I2C_Init(void);
void I2C_Write(uint8_t slave_addr, uint8_t *databuf, uint8_t length);
void I2C_Read(uint8_t slave_addr, uint8_t *databuf, uint8_t length);

#endif  // __I2C_H__

// === End of I2C.h ===

// === Start of Init.h ===
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
// === End of Init.h ===

// === Start of OP_amp.h ===
#ifndef __OP_H__
#define __OP_H__

/*_____ I N C L U D E S ____________________________________________________*/
#include <SN8F5602.h>


/*_____ D E F I N I T I O N S ______________________________________________*/
// OPM
#define OPN_IN_SHORT  (1<<7)

#define mskOPOEN      (1<<4)
#define OPP_GND       (1<<1)
#define mskOPEN       (1<<0)
/*_____ D E C L A R A T I O N S ____________________________________________*/

/*_____ M A C R O S ________________________________________________________*/


/*_____ F U N C T I O N S __________________________________________________*/
void OP_Amp_Init(void);



#endif  // __OP_H__
// === End of OP_amp.h ===

// === Start of power.h ===
#ifndef __POWER_H__
#define __POWER_H__

/*_____ I N C L U D E S ____________________________________________________*/
#include <SN8F5602.h>
#include "config.h"
#include "system.h"

/*_____ D E F I N I T I O N S ______________________________________________*/
typedef enum {
    PERIODIC_HEAT_PHASE,
    PERIODIC_HEAT_END_PHASE,
    PERIODIC_REST_PHASE,
    PERIODIC_SLOWDOWN_PHASE
} PeriodicHeatState;

typedef enum {
    PULSE_WIDTH_MIN              = 0,  // **Set PW0D to minimum width**
    PULSE_WIDTH_PERIODIC_START   = 1,  // **Set PW0D to 280 (first-time PERIODIC_HEATING)**
    PULSE_WIDTH_NO_CHANGE        = 2   // **Keep PW0D unchanged**
} PulseWidthSelect;

typedef enum {
    SHAKE_HOLD,
    SHAKE_DECREASE,
    SHAKE_INCREASE
} FrequencyShakeState;

#define SLOWDOWN_PWM_START_WIDTH  20    // 312.5ns / 31.25ns = 10 clks
#define SLOWDOWN_PWM_MAX_WIDTH    42    // 1.5us / 31.25ns = 48 clks
#define SLOWDOWN_PWM_PERIOD       640   // 20us / 31.25ns = 640 clks


#define HEATING_SYNC_AC   1  // Wait for AC low before starting
#define HEATING_IMMEDIATE 0  // Start immediately without AC synchronization

#define POWER_CALC_ENABLE  1
#define POWER_CALC_DISABLE 0

/*_____ D E C L A R A T I O N S ____________________________________________*/
extern bit f_heating_initialized;
extern bit f_power_measure_valid;
extern bit f_power_updated;
extern uint8_t level;
extern uint8_t periodic_AC_sync_cnt;
extern PeriodicHeatState periodic_heat_state;

/*_____ M A C R O S ________________________________________________________*/

/*_____ F U N C T I O N S __________________________________________________*/
void Measure_Base_Current(void);
void Power_read(void);
void reset_power_read_data(void);
void init_heating(uint8_t sync_ac_low, PulseWidthSelect pulse_width_select);
void stop_heating(void);
void Zero_Crossing_Task(void);
void Frequency_shake(void);

#endif  // __POWER_H__


// === End of power.h ===

// === Start of PWM.h ===
#ifndef __PWM_H__
#define __PWM_H__

/*_____ I N C L U D E S ____________________________________________________*/
#include <SN8F5602.h>


/*_____ D E C L A R A T I O N S ____________________________________________*/


/*_____ D E F I N I T I O N S ______________________________________________*/
// PW0M
#define mskPW0EN        (1<<7)
#define mskPW0RATE      (7<<4)
#define PW0_DIV1        (7<<4)
#define PW0_DIV2        (6<<4)
#define PW0_DIV4        (5<<4)
#define PW0_DIV8        (4<<4)
#define PW0_DIV16       (3<<4)
#define PW0_DIV32       (2<<4)
#define PW0_DIV64       (1<<4)
#define PW0_DIV128      (0<<4)

#define mskPW0CKS       (1<<3)
#define PW0_FCPU        (0<<3)
#define PW0_HOSC        (1<<3)

#define mskPW0DIR       (1<<2)
#define PW0_NORMAL      (0<<2)
#define PW0_INVERS      (1<<2)

#define mskPW0PO        (1<<1)
#define mskPWM0OUT      (1<<0)

// PW0M1
#define mskSFDL         (1<<7)
#define mskPGOUT        (1<<0)

// IEN3
#define mskEPW0         (1<<2)





/*_____ M A C R O S ________________________________________________________*/
#define PWM_INTERRUPT_ENABLE    IEN3 |= mskEPW0;
#define PWM_INTERRUPT_DISABLE   IEN3 &= ~mskEPW0;

/*_____ F U N C T I O N S __________________________________________________*/
void PWM_Init(void);

#endif  // __PWM_H__

// === End of PWM.h ===

// === Start of system.h ===
#ifndef __SYSTEM_H__
#define __SYSTEM_H__


/*_____ I N C L U D E S ____________________________________________________*/
#include <SN8F5602.h>


/*_____ D E F I N I T I O N S ______________________________________________*/
// CLKSEL
#define SYS_CLK_DIV1    (0<<0)
#define SYS_CLK_DIV2    (1<<0)
#define SYS_CLK_DIV4    (2<<0)
#define SYS_CLK_DIV8    (3<<0)
#define SYS_CLK_DIV16   (4<<0)
#define SYS_CLK_DIV32   (5<<0)
#define SYS_CLK_DIV64   (6<<0)
#define SYS_CLK_DIV128  (7<<0)

#define MAX_CNTDOWN_TIMERS 5 // 支援最多 5 個倒數計時器

typedef union {
    struct {
      uint8_t IGBT_overheat : 1;
      uint8_t TOP_overheat : 1;
      uint8_t IGBT_sensor_fault : 1;
      uint8_t TOP_sensor_fault : 1;
      uint8_t Pot_missing : 1;
      uint8_t Over_voltage : 1;
      uint8_t Low_voltage : 1;
      uint8_t Over_current : 1;
      uint8_t Voltage_quick_change : 1;
      uint8_t Current_quick_large : 1;
    } f;
    uint16_t all_flags; // 用於快速檢查所有標誌
} ErrorFlags;

typedef union {
    struct {
        uint8_t IGBT_heat_warning  : 1;
    } f;
    uint8_t all_flags;
} WarningFlags;





typedef enum {
  TASK_HEAT_CONTROL,            // 加熱控制任務  
  TASK_POWER_CONTROL,           // 功率控制任務
  TASK_QUICK_CHANGE_DETECT,     // 快速變化檢測任務
  
  TASK_TEMP_MEASURE,            // 溫度測量任務
  TASK_TEMP_PROCESS,            // 溫度處理任務
    
  TASK_CURRENT_POT_CHECK,       // 電流檢鍋任務
  TASK_SHUTDOWN,                // 關機任務
  TASK_ERROR_PROCESS            // 錯誤處理任務
} TaskType;

typedef enum {
    STANDBY = 0,          // 待機狀態
    HEATING,              // 加熱中
    PERIODIC_HEATING,     // 間歇加熱中
    SHUTTING_DOWN,        // 關機狀態
    PAUSE,
    ERROR,
} SystemState;


typedef enum {
    POT_IDLE,
    POT_CHECKING,
    POT_ANALYZING
} PotDetectionState;

typedef enum {
    PWR_UP,
    WAIT_STABILIZATION,
    RECORDING
} PotAnalyzeState;


#define CNTDOWN_TIMER_POT_DETECT                  0 // 鍋具檢測間隔計時器id
#define CNTDOWN_TIMER_POT_HEATING_CURRENT_DELAY   1 // 控制 16ms 電流變化檢查倒數計時器 ID
#define CNTDOWN_TIMER_POWER_CONTROL               2 // 控制 5ms POWER_CONTROL
#define CNTDOWN_TIMER_I2C                         3 // I2C 計時器 ID

/*_____ D E C L A R A T I O N S ____________________________________________*/
extern volatile bit f_125us;
extern volatile uint8_t system_ticks;
extern volatile uint8_t pot_pulse_cnt;   // 鍋具脈衝計數器

extern ErrorFlags error_flags;
extern WarningFlags warning_flags;
extern volatile uint8_t Surge_Overvoltage_Flag; // By CM1
extern volatile uint8_t Surge_Overcurrent_Flag; // By CM4

extern SystemState system_state;
extern uint32_t power_setting;
extern uint32_t target_power;
extern TaskType current_task; // 當前任務
//extern uint16_t voltage_IIR_new;         // 目前濾波後電壓
//extern uint16_t current_IIR_new;         // 目前濾波後電流
extern uint32_t current_power;           // 目前功率
extern uint16_t current_RMS_mA;
extern uint16_t voltage_RMS_V;
extern uint16_t current_base;  // 存儲基準電流 ADC 值

extern uint16_t recorded_1000W_PW0D;
extern uint8_t ac_half_low_ticks_avg;

extern bit f_pot_detected;
extern bit f_AC_50Hz; // **AC 頻率標誌，1 = 50Hz，0 = 60Hz**
extern uint8_t measure_per_AC_cycle;

extern bit f_block_occurred;

extern PotDetectionState pot_detection_state;
extern PotAnalyzeState pot_analyze_state;

//DEBUG
extern xdata uint16_t tune_cnt;
extern xdata uint32_t tune_record1;
extern xdata uint32_t tune_record2;

/*_____ M A C R O S ________________________________________________________*/

/*_____ F U N C T I O N S __________________________________________________*/
void SystemCLK_Init(void);
void CNTdown_Timer_Init(void);
uint8_t cntdown_timer_expired(uint8_t timer_id);
void cntdown_timer_start(uint8_t timer_id, uint16_t duration);

void Update_System_Time(void);
void shutdown_process(void);

void Quick_Change_Detect(void);
void Heat_Control(void);
void pause_heating(void);
void Periodic_Power_Control(void);
void Power_Control(void);
void Pot_Detection(void);
void Pot_Analyze(void);
void Pot_Detection_In_Heating(void);
void Shutdown_Task(void);
void Measure_AC_Low_Time(void);
void Detect_AC_Frequency(void);
void Error_Process(void);

#endif  // __SYSTEM_H__

// === End of system.h ===

// === Start of temperature.h ===
#ifndef __TEMP_H__
#define __TEMP_H__

/*_____ I N C L U D E S ____________________________________________________*/
#include <SN8F5602.h>

/*_____ D E F I N I T I O N S ______________________________________________*/

/*_____ D E C L A R A T I O N S ____________________________________________*/
extern int idata IGBT_TEMP_C;      // 目前IGBT溫度
extern int idata TOP_TEMP_C;       // 目前表面溫度

/*_____ M A C R O S ________________________________________________________*/

/*_____ F U N C T I O N S __________________________________________________*/
void Temp_Measure(void );
void Temp_Process(void);


#endif  // __TEMP_H__
// === End of temperature.h ===

// === Start of timer0.h ===
#ifndef __TIMER_H__
#define __TIMER_H__

/*_____ I N C L U D E S ____________________________________________________*/
#include <SN8F5602.h>


/*_____ D E F I N I T I O N S ______________________________________________*/
// T0M
#define mskT0EN       (1<<7)
#define DIV_1         (7<<4)
#define DIV_2         (6<<4)
#define DIV_4         (5<<4)
#define DIV_8         (4<<4)
#define DIV_16        (3<<4)
#define DIV_32        (2<<4)
#define DIV_64        (1<<4)
#define DIV_128       (0<<4)

#define CLK_FCPU      (0<<2)
#define CLK_FHOSC     (2<<2)
#define CLK_FLOSC     (3<<2)

// TCON
#define mskTF0         (1<<5)


/*_____ D E C L A R A T I O N S ____________________________________________*/

/*_____ M A C R O S ________________________________________________________*/


/*_____ F U N C T I O N S __________________________________________________*/
void Timer0_Init(void);


#endif  // __TIMER_H__
// === End of timer0.h ===

