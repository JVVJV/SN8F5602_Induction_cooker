#ifndef __COMPARATOR_H__
#define __COMPARATOR_H__

/*_____ I N C L U D E S ____________________________________________________*/
#include <SN8F5602.h>


/*_____ D E F I N I T I O N S ______________________________________________*/
// CM0M
#define mskCM0EN        (1<<7)
#define mskCM0SF        (1<<4)

#define CM0_RISING_TRIGGER    (1<<3)
#define CM0_FALLING_TRIGGER   (0<<3)
#define CM0_CLK_HOSC      (0<<2)
#define CM0_CLK_FCPU      (1<<2)
// CMDB0
#define DELAY_0T       (0<<4)
#define DELAY_4T       (1<<4)
#define DELAY_8T       (2<<4)
#define DELAY_12T      (3<<4)
#define DELAY_16T      (4<<4)
#define DELAY_20T      (5<<4)
#define DELAY_24T      (6<<4)
#define DELAY_28T      (7<<4)
#define DELAY_32T      (8<<4)
#define DELAY_36T      (9<<4)
#define DELAY_40T      (10<<4)
#define DELAY_44T      (11<<4)
#define DELAY_48T      (12<<4)
#define DELAY_52T      (13<<4)
#define DELAY_56T      (14<<4)
#define DELAY_60T      (15<<4)

#define DEBOUNCE_0FCPU      (0<<0)
#define DEBOUNCE_2FCPU      (1<<0)
#define DEBOUNCE_4FCPU      (2<<0)
#define DEBOUNCE_6FCPU      (3<<0)
#define DEBOUNCE_8FCPU      (4<<0)
#define DEBOUNCE_10FCPU     (5<<0)
#define DEBOUNCE_12FCPU     (6<<0)
#define DEBOUNCE_14FCPU     (7<<0)
#define DEBOUNCE_16FCPU     (8<<0)
#define DEBOUNCE_18FCPU     (9<<0)
#define DEBOUNCE_20FCPU     (10<<0)
#define DEBOUNCE_22FCPU     (11<<0)
#define DEBOUNCE_24FCPU     (12<<0)
#define DEBOUNCE_26FCPU     (13<<0)
#define DEBOUNCE_28FCPU     (14<<0)
#define DEBOUNCE_30FCPU     (15<<0)

// CM1M
#define mskCM1EN        (1<<7)
#define mskCM1SF        (1<<4)

#define CM1_RISING_TRIGGER    (1<<3)
#define CM1_FALLING_TRIGGER   (0<<3)
#define CM1_CLK_HOSC          (0<<2)
#define CM1_CLK_FCPU          (1<<2)

// CM1REF
#define CM1REF_VDD      (0<<7)
#define CM1REF_INTREF   (1<<7)

// CMDB1
#define CM1_DEBOUNCE_0FCPU      (0<<0)
#define CM1_DEBOUNCE_2FCPU      (1<<0)
#define CM1_DEBOUNCE_4FCPU      (2<<0)
#define CM1_DEBOUNCE_6FCPU      (3<<0)
#define CM1_DEBOUNCE_8FCPU      (4<<0)
#define CM1_DEBOUNCE_10FCPU     (5<<0)
#define CM1_DEBOUNCE_12FCPU     (6<<0)
#define CM1_DEBOUNCE_14FCPU     (7<<0)
#define CM1_DEBOUNCE_16FCPU     (8<<0)
#define CM1_DEBOUNCE_18FCPU     (9<<0)
#define CM1_DEBOUNCE_20FCPU     (10<<0)
#define CM1_DEBOUNCE_22FCPU     (11<<0)
#define CM1_DEBOUNCE_24FCPU     (12<<0)
#define CM1_DEBOUNCE_26FCPU     (13<<0)
#define CM1_DEBOUNCE_28FCPU     (14<<0)
#define CM1_DEBOUNCE_30FCPU     (15<<0)

// CM2M
#define mskCM2EN    (1<<7)
#define mskCM2SF    (1<<4)

#define CM2_RISING_TRIGGER    (1<<3)
#define CM2_FALLING_TRIGGER   (0<<3)
// CMDB2

// CM2REF
#define CM2REF_VDD      (0<<7)
#define CM2REF_INTREF   (1<<7)


// CM3M
#define mskCM3EN    (1<<7)

#define CM3_RISING_TRIGGER    (1<<3)
#define CM3_FALLING_TRIGGER   (0<<3)
// CMDB3
#define CM3_DEBOUNCE_0FCPU      (0<<0)
#define CM3_DEBOUNCE_2FCPU      (1<<0)
#define CM3_DEBOUNCE_4FCPU      (2<<0)
#define CM3_DEBOUNCE_6FCPU      (3<<0)
#define CM3_DEBOUNCE_8FCPU      (4<<0)
#define CM3_DEBOUNCE_10FCPU     (5<<0)
#define CM3_DEBOUNCE_12FCPU     (6<<0)
#define CM3_DEBOUNCE_14FCPU     (7<<0)
#define CM3_DEBOUNCE_16FCPU     (8<<0)
#define CM3_DEBOUNCE_18FCPU     (9<<0)
#define CM3_DEBOUNCE_20FCPU     (10<<0)
#define CM3_DEBOUNCE_22FCPU     (11<<0)
#define CM3_DEBOUNCE_24FCPU     (12<<0)
#define CM3_DEBOUNCE_26FCPU     (13<<0)
#define CM3_DEBOUNCE_28FCPU     (14<<0)
#define CM3_DEBOUNCE_30FCPU     (15<<0)

// CM3REF
#define CM3REF_VDD      (0<<7)
#define CM3REF_INTREF   (1<<7)

// CMOUT
#define mskCM4OUT     (1<<4)
#define mskCM3OUT     (1<<3)
#define mskCM2OUT     (1<<2)
#define mskCM1OUT     (1<<1)
#define mskCM0OUT     (1<<0)

// INTREF
#define mskINTREFEN   (1<<7)
#define INTREF2V5     (0<<0)
#define INTREF3V5     (1<<0)
#define INTREF4V5     (2<<0)
#define INTREF1V5     (3<<0)

// TCON
#define mskTF0      (1<<5)

// IEN2
#define mskECMP4    (1<<7)
#define mskECMP3    (1<<6)
#define mskECMP2    (1<<5)
#define mskECMP1    (1<<4)
#define mskECMP0    (1<<3)

// IRCON2
#define mskCM0F     (1<<3)

/*_____ D E C L A R A T I O N S ____________________________________________*/
extern volatile bit f_CM3_AC_sync;

/*_____ M A C R O S ________________________________________________________*/
#define CM0_IRQ_ENABLE  IEN2 |= mskECMP0
#define CM0_IRQ_DISABLE IEN2 &= (~mskECMP0)

#define CLEAR_CM0_IRQ_FLAG IRCON2 &= (~mskCM0F)

/*_____ F U N C T I O N S __________________________________________________*/
void Comparator_Init(void);


#endif  // __COMPARATOR_H__
