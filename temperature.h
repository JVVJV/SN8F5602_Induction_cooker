#ifndef __TEMP_H__
#define __TEMP_H__

/*_____ I N C L U D E S ____________________________________________________*/
#include <SN8F5602.h>

/*_____ D E F I N I T I O N S ______________________________________________*/

/*_____ D E C L A R A T I O N S ____________________________________________*/
extern int IGBT_TEMP_C;      // 目前IGBT溫度
extern int TOP_TEMP_C;       // 目前表面溫度

/*_____ M A C R O S ________________________________________________________*/

/*_____ F U N C T I O N S __________________________________________________*/
void Temp_Measure(void );
void Temp_Process(void);


#endif  // __TEMP_H__