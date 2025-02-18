#ifndef __BUZZER_H__
#define __BUZZER_H__

/*_____ I N C L U D E S ____________________________________________________*/
#include <SN8F5602.h>


/*_____ D E C L A R A T I O N S ____________________________________________*/


/*_____ D E F I N I T I O N S ______________________________________________*/
// PW0M
#define mskBZEN        (1<<7)
#define BZ_RATE_512    (0<<4)
#define BZ_RATE_1024   (1<<4)
#define BZ_RATE_2048   (2<<4)
#define BZ_RATE_4096   (3<<4)
#define BZ_RATE_8192   (4<<4)
#define BZ_RATE_16384  (5<<4)
#define BZ_RATE_32768  (6<<4)
#define BZ_RATE_65536  (7<<4)


/*_____ M A C R O S ________________________________________________________*/
#define BUZZER_ENABLE   BZM |= mskBZEN
#define BUZZER_DISABLE   BZM &= (~mskBZEN)

/*_____ F U N C T I O N S __________________________________________________*/
void Buzzer_Init(void);

#endif  // __PWM_H__
