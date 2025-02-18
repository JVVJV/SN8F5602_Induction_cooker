#ifndef __POWER_H__
#define __POWER_H__

/*_____ I N C L U D E S ____________________________________________________*/
#include <SN8F5602.h>
#include "config.h"
#include "system.h"

/*_____ D E F I N I T I O N S ______________________________________________*/



/*_____ D E C L A R A T I O N S ____________________________________________*/
extern bit f_En_check_current_change;
extern bit f_heating_initialized;
/*_____ M A C R O S ________________________________________________________*/

/*_____ F U N C T I O N S __________________________________________________*/
void Power_read(void);
void init_heating(void);

#endif  // __POWER_H__

