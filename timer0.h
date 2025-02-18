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