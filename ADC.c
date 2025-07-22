/******************** (C) COPYRIGHT 2024 SONiX *******************************
* COMPANY:	SONiX
* DATE:		  2025/01
* AUTHOR:		HCW
* IC:			  SR56F27
*____________________________________________________________________________
* REVISION		Date				User		Description
*____________________________________________________________________________
*****************************************************************************/

/*_____ I N C L U D E S ____________________________________________________*/
#include "ADC.h"
#include "config.h"

/*_____ D E C L A R A T I O N S ____________________________________________*/


/*_____ D E F I N I T I O N S ______________________________________________*/


/*_____ M A C R O S ________________________________________________________*/

/*_____ F U N C T I O N S __________________________________________________*/
void ADC_Init(void)
{
  ADM = 0;
  // 32MHz/2 * 16T = 1MHz 
  ADR = ADC_CLK_HOSC | mskGCHS;
  VREFH = VREF_INTERNAL | VREF_VDD;
  ADCAL = SAMPLE_3T5 | ADC_CLK_DIV2;

  ADM |= mskADENB|VOLTAGE_ADC_CHANNEL;
  // ADC calibration
  ADCAL |= mskACS;
  while((ADCAL&mskACS) != 0);
  ADCAL |= mskCALIVALENB;
}



void select_ADC_channel(uint8_t channel) {
  ADM =  (ADM & ~0x1F) | channel;
}

void ADC_measure_4_avg(uint8_t channel, uint16_t *result) {
    //uint16_t samples[4];  // Buffer for 4 measurement results
    uint16_t sum = 0;
    uint8_t i;

    // Switch to the specified ADC channel
    select_ADC_channel(channel);
  
    // Measure four times continuously
    for (i = 0; i < 4; i++) {
        START_ADC_CONVERSION;
        while (IS_ADC_FINISH == 0);
        CLEAR_EOC;
        sum += GET_ADC_RESULT;  //  Get ADC result
    }
    
//    // Find the maximum and minimum values and compute the sum
//    max_value = samples[0];
//    min_value = samples[0];
//    sum = samples[0];
//    for (i = 1; i < 4; i++) {  //Start from index 1
//        if (samples[i] > max_value) {
//            max_value = samples[i];
//        } else if (samples[i] < min_value) {
//            min_value = samples[i];
//        }
//        sum += samples[i];
//    }
//    // Remove max and min, then average the middle two
//    sum = sum - max_value - min_value;
    
    *result = sum / 4;  //  Return result to the specified variable addre
    
}




