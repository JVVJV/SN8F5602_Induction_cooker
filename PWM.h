#ifndef __PWM_H__
#define __PWM_H__

/*_____ I N C L U D E S ____________________________________________________*/
#include <SN8F5602.h>

/*_____ D E C L A R A T I O N S ____________________________________________*/

/*_____ D E F I N I T I O N S ______________________________________________*/
// PW0M
#define mskPW0EN        (1<<7)
#define mskPW0RATE      (7<<4)
#define PW0_DIV1        (7<<4)
#define PW0_DIV2        (6<<4)
#define PW0_DIV4        (5<<4)
#define PW0_DIV8        (4<<4)
#define PW0_DIV16       (3<<4)
#define PW0_DIV32       (2<<4)
#define PW0_DIV64       (1<<4)
#define PW0_DIV128      (0<<4)

#define mskPW0CKS       (1<<3)
#define PW0_FCPU        (0<<3)
#define PW0_HOSC        (1<<3)

#define mskPW0DIR       (1<<2)
#define PW0_NORMAL      (0<<2)
#define PW0_INVERS      (1<<2)

#define mskPW0PO        (1<<1)
#define mskPWM0OUT      (1<<0)

// PW0M1
#define mskSFDL         (1<<7)
#define mskPGOUT        (1<<0)

// IEN3
#define mskEPW0         (1<<2)
// IRCON3
#define mskPW0F         (1<<2)


/*_____ M A C R O S ________________________________________________________*/
#define PWM_INTERRUPT_ENABLE    IEN3 |= mskEPW0;
#define PWM_INTERRUPT_DISABLE   IEN3 &= ~mskEPW0;
#define PW0F_CLEAR              IRCON3 &= ~mskPW0F;

/*_____ F U N C T I O N S __________________________________________________*/
void PWM_Init(void);


#endif  // __PWM_H__
