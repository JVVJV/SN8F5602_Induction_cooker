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