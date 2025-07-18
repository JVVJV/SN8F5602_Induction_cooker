#ifndef __TIMER1_H__
#define __TIMER1_H__

/*_____ I N C L U D E S ____________________________________________________*/
#include <SN8F5602.h>

/*_____ D E F I N I T I O N S ______________________________________________*/
// T1M
#define mskT1EN       (1<<7)
#define T1_DIV_1      (7<<4)
#define T1_DIV_2      (6<<4)
#define T1_DIV_4      (5<<4)
#define T1_DIV_8      (4<<4)
#define T1_DIV_16     (3<<4)
#define T1_DIV_32     (2<<4)
#define T1_DIV_64     (1<<4)
#define T1_DIV_128    (0<<4)

#define T1_CLK_FCPU   (0<<3)
#define T1_CLK_FHOSC  (1<<3)

#define mskT1SF       (1<<0)

// TCON
#define mskTF1        (1<<7)


/*_____ D E C L A R A T I O N S ____________________________________________*/

/*_____ M A C R O S ________________________________________________________*/

/*_____ F U N C T I O N S __________________________________________________*/
void Timer1_Init(void);
void Timer1_Enable(void);
void Timer1_Disable(void);


#endif  // __TIMER1_H__
