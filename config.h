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
#define AC_FREQ_MODE_FORCE_50HZ   0
#define AC_FREQ_MODE_FORCE_60HZ   1
// Select desired AC frequency mode here:
#define AC_FREQ_MODE  AC_FREQ_MODE_FORCE_60HZ


#define CURRENT_ADC_CHANNEL       19  // OPO  定義系統電流量測的 ADC 通道 
#define VOLTAGE_ADC_CHANNEL       7   // AIN7 定義電網電壓量測的 ADC 通道
#define IGBT_TEMP_ADC_CHANNEL     1   // AIN1
#define TOP_TEMP_ADC_CHANNEL      2   // AIN2

#define PERIODIC_TARGET_POWER     1000000 // 1000 000mW




//#define PWM_MAX_WIDTH           960         // PWM 最大寬度   960cnt @32MHz = 30us
//#define PWM_MAX_WIDTH           896         // PWM 最大寬度   896cnt @32MHz = 28us
//#define PWM_MAX_WIDTH           768         // PWM 最大寬度   768cnt @32MHz = 24us
//#define PWM_MAX_WIDTH           704         // PWM 最大寬度   704cnt @32MHz = 22us
#define PWM_MAX_WIDTH           640         // PWM 最大寬度   640cnt @32MHz = 20us
//#define PWM_MAX_WIDTH           512         // PWM 最大寬度   512cnt @32MHz = 16us
//#define PWM_MAX_WIDTH           417         // PWM 最大寬度   417cnt @32MHz = 13us
//#define PWM_MAX_WIDTH           320         // PWM 最大寬度   320cnt @32MHz = 10us
//#define PWM_MAX_WIDTH           256         // PWM 最大寬度   250cnt @32MHz = 8us


#define PWM_INIT_WIDTH          224         // PWM init_heating 224cnt @32MHz = 7us

#define POT_DETECT_PULSE_TIME   192         // 設定 PW0D 數值為檢鍋用值 (6us)

#define PWM_MIN_WIDTH           208         // PWM 最小寬度   208cnt @32MHz = 6.5us
                                            // This value should not be too small, as a smaller value 
                                            // may increase the proportion of hard-switching operation, 
                                            // leading to IGBT overheating.

#define	I2C_SLAVE_ADDRESS					0x55
#define I2C_INTERVAL   43    // **I2C 操作間隔 43ms**

#endif  // __CONFIG_H__