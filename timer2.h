#ifndef __TIMER2_H__
#define __TIMER2_H__

/*_____ I N C L U D E S ____________________________________________________*/
#include <SN8F5602.h>

/*_____ D E F I N I T I O N S ______________________________________________*/
// T2M
#define mskT2EN       (1<<7)
#define mskT2SF       (1<<0)
#define T2_DIV_1      (7<<4)
#define T2_DIV_2      (6<<4)
#define T2_DIV_4      (5<<4)
#define T2_DIV_8      (4<<4)
#define T2_DIV_16     (3<<4)
#define T2_DIV_32     (2<<4)
#define T2_DIV_64     (1<<4)
#define T2_DIV_128    (0<<4)

#define T2_CLK_FCPU   (0<<3)
#define T2_CLK_FHOSC  (1<<3)

// IRCON2
#define mskTF2        (1<<2)


/*_____ D E C L A R A T I O N S ____________________________________________*/

/*_____ M A C R O S ________________________________________________________*/

/*_____ F U N C T I O N S __________________________________________________*/
void Timer2_Init(void);


#endif  // __TIMER2_H__
