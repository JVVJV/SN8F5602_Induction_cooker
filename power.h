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
    JITTER_HOLD,            // Initial idle state before jitter starts
    JITTER_DECREASE,        // Frequency jitter decreasing phase, PWM0_ISR will decrement PW0D
    JITTER_INCREASE,        // Frequency jitter increasing phase, PWM0_ISR will increment PW0D
} FrequencyJitterState;


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
extern FrequencyJitterState Frequency_jitter_state;
extern volatile uint8_t jitter_adjust_cnt;
extern bit f_jitter_active;
extern bit f_power_switching;
extern bit f_orig_power_set;
extern volatile bit PW0D_req_quick_surge;


/*_____ M A C R O S ________________________________________________________*/

/*_____ F U N C T I O N S __________________________________________________*/
void Measure_Base_Current(void);
void Power_read(void);
void reset_power_read_data(void);
void init_heating(uint8_t sync_ac_low, PulseWidthSelect pulse_width_select);
void stop_heating(void);
void Zero_Crossing_Task(void);
void Frequency_jitter(void);

#endif  // __POWER_H__

