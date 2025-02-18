#ifndef __ADC_H__
#define __ADC_H__

/*_____ I N C L U D E S ____________________________________________________*/
#include <SN8F5602.h>


/*_____ D E C L A R A T I O N S ____________________________________________*/


/*_____ D E F I N I T I O N S ______________________________________________*/
// ADM
#define mskADENB        (1<<7)
#define mskADS          (1<<6)
#define mskEOC          (1<<5)
// ADR
#define ADC_CLK_HOSC    (0<<7)
#define ADC_CLK_LOSC    (1<<7)
#define mskGCHS         (1<<6)
// VREFH
#define VREF_INTERNAL   (0<<7)
#define VREF_EXTERNAL   (1<<7)
#define VREF_VDD        (4<<0)
#define VREF_2V5        (0<<0)
#define VREF_3V5        (1<<0)
#define VREF_4V5        (2<<0)
#define VREF_1V5        (3<<0)
// ADCAL
#define mskACS          (1<<7)
#define mskCALIVALENB   (1<<6)
#define SAMPLE_3T5      (0<<4)
#define SAMPLE_4T5      (1<<4)
#define SAMPLE_6T5      (2<<4)
#define SAMPLE_10T5     (3<<4)
#define ADC_CLK_DIV1    (0<<0)
#define ADC_CLK_DIV2    (1<<0)
#define ADC_CLK_DIV4    (2<<0)
#define ADC_CLK_DIV8    (3<<0)
#define ADC_CLK_DIV16   (4<<0)
#define ADC_CLK_DIV32   (5<<0)
#define ADC_CLK_DIV64   (6<<0)
#define ADC_CLK_DIV128  (7<<0)




/*_____ M A C R O S ________________________________________________________*/
#define START_ADC_CONVERSION  (ADM |= mskADS)
#define IS_ADC_FINISH   (ADM&mskEOC)
#define CLEAR_EOC       (ADM &= ~mskEOC)
#define GET_ADC_RESULT  ((ADB<<4)|(ADR&0x0F))

/*_____ F U N C T I O N S __________________________________________________*/
void ADC_Init(void);
void ADC_measure_4_avg(uint8_t channel, uint16_t *result);

#endif  // __ADC_H__
