#ifndef __POWER_H__
#define __POWER_H__

/*_____ I N C L U D E S ____________________________________________________*/
#include <SN8F5602.h>
#include "config.h"
#include "system.h"

/*_____ D E F I N I T I O N S ______________________________________________*/
typedef enum {
    NORMAL,
    PERIODIC
} HeatingMode;

typedef enum {
    PERIODIC_HEAT_PHASE,
    PERIODIC_HEAT_END_PHASE,  // **確保最後一個加熱週期結束後，先等 `ac_half_low_ticks_avg`**
    PERIODIC_REST_PHASE,
    PERIODIC_SLOWDOWN_PHASE
} PeriodicHeatState;

/*_____ D E C L A R A T I O N S ____________________________________________*/
extern bit f_En_check_current_change;
extern bit f_heating_initialized;
extern uint8_t level;
extern volatile PeriodicHeatState periodic_heat_state;

/*_____ M A C R O S ________________________________________________________*/

/*_____ F U N C T I O N S __________________________________________________*/
void Power_read(void);
void init_heating(HeatingMode heating_mode);

#endif  // __POWER_H__

