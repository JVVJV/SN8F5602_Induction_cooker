#ifndef __TEMP_H__
#define __TEMP_H__

/*_____ I N C L U D E S ____________________________________________________*/
#include <SN8F5602.h>

/*_____ D E F I N I T I O N S ______________________________________________*/

/*_____ D E C L A R A T I O N S ____________________________________________*/
extern int idata IGBT_TEMP_C;      // IGBT temperature
extern int idata TOP_TEMP_C;       // Surface temperature
extern bit    f_ntc_monitoring;
extern uint8_t ntc_change_count;
extern idata int last_IGBT_temp;

/*_____ M A C R O S ________________________________________________________*/

/*_____ F U N C T I O N S __________________________________________________*/
void Temp_Measure(void );
void Temp_Process(void);
//void Print_All_IGBT_Temps(void);  //HCW** 


#endif  // __TEMP_H__