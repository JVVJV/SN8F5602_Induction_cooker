#ifndef __CONFIG_H__
#define __CONFIG_H__

/*_____ I N C L U D E S ____________________________________________________*/
#include <SN8F5602.h>

/*_____ D E C L A R A T I O N S ____________________________________________*/

/*_____ D E F I N I T I O N S ______________________________________________*/
#define ICE_DEBUG_MODE      1
#define TUNE_MODE           1

// ----------------------------------------------------------------------------------------
#define COIL_PROBLEM_DETECT     1
#define UNEXPECTED_HALT_DETECT  1
#define BELOWRATED_VOLTAGE_CONSTANT_CURRENT_MODE  1

// ----------------------------------------------------------------------------------------
#define MEASUREMENTS_PER_60HZ 133     // Number of measurements per full 60 Hz cycle (based on 125us)
#define MEASUREMENTS_PER_50HZ 160     // Number of measurements per full 50 Hz cycle (based on 125us)

// AC Frequency Mode Control
#define AC_FREQ_MODE_FORCE_50HZ   0
#define AC_FREQ_MODE_FORCE_60HZ   1
// Select desired AC frequency mode here:
#define AC_FREQ_MODE  AC_FREQ_MODE_FORCE_50HZ

// ----------------------------------------------------------------------------------------
// Burstmode Control
#define BURST_MODE_BASIC         0   // Heat & rest cycle combination is fixed
#define BURST_MODE_DYNAMIC       1   // Heat & rest cycle combination switches dynamically
// Select desired AC burst mode here:
#define BURST_MODE  BURST_MODE_DYNAMIC


#define PERIODIC_TARGET_POWER     1000000 // 1000 000mW

// ----------------------------------------------------------------------------------------
#define CURRENT_ADC_CHANNEL       19  // OPO  defines ADC channel for current measurement 
#define VOLTAGE_ADC_CHANNEL       7   // AIN7 defines ADC channel for AC mains voltage measurement
#define IGBT_TEMP_ADC_CHANNEL     1   // AIN1
#define TOP_TEMP_ADC_CHANNEL      2   // AIN2

// ----------------------------------------------------------------------------------------
//#define PWM_MAX_WIDTH           960       // Max PWM width: 960cnt @32MHz = 30us
//#define PWM_MAX_WIDTH           896       // Max PWM width: 896cnt @32MHz = 28us
//#define PWM_MAX_WIDTH           768       // Max PWM width: 768cnt @32MHz = 24us 
#define PWM_MAX_WIDTH             704       // Max PWM width: 704cnt @32MHz = 22us
//#define PWM_MAX_WIDTH           640       // Max PWM width: 640cnt @32MHz = 20us
//#define PWM_MAX_WIDTH           512       // Max PWM width: 512cnt @32MHz = 16us
//#define PWM_MAX_WIDTH           417       // Max PWM width: 417cnt @32MHz = 13us
//#define PWM_MAX_WIDTH           320       // Max PWM width: 320cnt @32MHz = 10us
//#define PWM_MAX_WIDTH           256       // Max PWM width: 250cnt @32MHz = 8us

#define PWM_INIT_WIDTH            224       // PWM init_heating 224cnt @32MHz = 7us

#define POT_DETECT_PULSE_WIDTH    192       // 設定 PW0D 數值為檢鍋用值 (6us)

#define PWM_MIN_WIDTH             208       // PWM 最小寬度   208cnt @32MHz = 6.5us
                                            // This value should not be too small, as a smaller value 
                                            // may increase the proportion of hard-switching operation, 
                                            // leading to IGBT overheating.
                                            
// ----------------------------------------------------------------------------------------
#define VOLTAGE_CHANGE_THRESHOLD 20     // Voltage change > 20V is considered a rapid change
#define CURRENT_CHANGE_THRESHOLD 1000   // Current change > 1000mA is considered a rapid change
#define QUICK_SURGE_MODIFY_WIDTH 10

// ----------------------------------------------------------------------------------------
#define T2SF_MARGIN     64      // 2us @ 32MHz
#define T2SF_RELOAD     (uint16_t)(0xFFFF - (PWM_MAX_WIDTH+T2SF_MARGIN+1))

// ----------------------------------------------------------------------------------------
#define	I2C_SLAVE_ADDRESS	  0x55
#define I2C_INTERVAL        43          // I2C operation interval: 43 ms

/*_____ M A C R O S ________________________________________________________*/

/*_____ F U N C T I O N S __________________________________________________*/


#endif  // __CONFIG_H__
