/******************** (C) COPYRIGHT 2024 SONiX *******************************
* COMPANY:	SONiX
* DATE:		  2025/01
* AUTHOR:		HCW
* IC:			  SR56F27
*____________________________________________________________________________
* REVISION		Date				User		Description
*____________________________________________________________________________
*****************************************************************************/
/**
 * @file comparator.c
 * @brief Comparator initialization and ISRs for surge and AC sync handling.
 */

/*_____ I N C L U D E S ____________________________________________________*/
#include "comparator.h"
#include "config.h"
#include "system.h"
#include "power.h"
#include "PWM.h"

/*_____ D E C L A R A T I O N S ____________________________________________*/

/*_____ D E F I N I T I O N S ______________________________________________*/

/*_____ M A C R O S ________________________________________________________*/

/*_____ F U N C T I O N S __________________________________________________*/
/**
 * @brief Initialize comparators CM0~CM4 and internal reference voltage.
 *
 * Configures pin modes, thresholds, debouncing, triggers, and enables interrupts
 * for five comparators:
 * - CM0: PWM pulse trigger
 * - CM1: AC overvoltage surge protection
 * - CM2: IGBT overvoltage protection
 * - CM3: AC zero-cross synchronization
 * - CM4: Current overcurrent surge protection
 */
void Comparator_Init(void)
{
  // ------------------------------------------------------------------------
  // CM0: PWM pulse trigger
  // ------------------------------------------------------------------------
  CM0M = 0;
  // P02, P03 switch to analog pin
  P0CON |= (1<<3)|(1<<2); 
  
  CM0M = CM0_FALLING_TRIGGER | CM0_CLK_FCPU;
  CMDB0 = DELAY_4T | DEBOUNCE_16FCPU;
  CM0M |= mskCM0EN;
  
  // ------------------------------------------------------------------------
  // Internal Reference Voltage
  // ------------------------------------------------------------------------
  INTREF = mskINTREFEN | INTREF3V5;
  
  // ------------------------------------------------------------------------
  // CM1: AC overvoltage surge protection (@260 Vrms)
  // ------------------------------------------------------------------------
  // Protection voltage is 260V RMS (peak 367V) 
  // with a resistor divider ratio of 0.0049, resulting in 1.8V.
  
  // After the induction cooker begins operation, the GND potential shifts upward & down, so we need an increase in the voltage surge detection threshold.
  // For this prototype circuit, an upward adjustment of 225mV is required
  CM1M = 0;
  
  // CM1N, P04 switch to analog pin
  P0CON |= (1<<4); 
  
  // CM1P
  // Set CM1+ internal-reference 1.8V + 0.225V = 2.025V
  // INTERNAL 3.5V*37/64 = 2.023V-> CM1RS = 37
  CM1REF = CM1REF_INTREF | 37;
  
  CMDB1 = CM1_DEBOUNCE_10FCPU; // 0.625us
  CM1M = mskCM1EN | mskCM1SF |CM1_FALLING_TRIGGER;
  IEN2 |= mskECMP1;
  
  
  // ------------------------------------------------------------------------
  // CM2: IGBT overvoltage protection (@1050V) (AC power High or Pot be taken off)
  // ------------------------------------------------------------------------
  CM2M = 0;
  // CM2N, P05 switch to analog pin
  P0CON |= (1<<5);  
  
  // CM2P
  // 1050V*2.5K/827.5K = 3.1722V
  // INTERNAL 3.5V*58/64 = 3.1718V -> CM2RS = 58 
  CM2REF = CM2REF_INTREF | 58;    //1050V
  
  //CM2REF = CM2REF_INTREF | 38;  //700V
  //CM2REF = CM2REF_INTREF | 44;  //800V
  //CM2REF = CM2REF_INTREF | 50;  //900V
  //CM2REF = CM2REF_INTREF | 55;  //1000V
  
  CMDB2 = CM2_DEBOUNCE_4FCPU; // 0.25us
  CM2M = mskCM2EN |CM2_FALLING_TRIGGER; // Disable CM2SF HCW***
  IEN2 |= mskECMP2;
  
  
  // ------------------------------------------------------------------------
  // CM3: AC power synchronization
  // ------------------------------------------------------------------------
  CM3M = 0;
  
  // CM3P
  // P04 switch to analog pin
  // P0CON |= (1<<4);    // It has already been executed during CM2 initialization.
  
  // CM3N
  // Set CM3- to internal reference voltage 0.5V
  // INTERNAL 3.5V*9/64 = 0.5V (0.49V) -> CM3RS = 9 
  CM3REF = CM3REF_INTREF | 9;
  
  //CM3REF = CM3REF_INTREF | 1; // 54mV
  
  CMDB3 = CM3_DEBOUNCE_20FCPU;
  CM3M = mskCM3EN | CM3_FALLING_TRIGGER;
  IEN2 |= mskECMP3;
  
  // ------------------------------------------------------------------------
  // CM4: Current overcurrent surge protection (~10A)
  // ------------------------------------------------------------------------
  // For the prototype, the amplification ratio is approximately 47K/1.25K = 37.6x. With a 10A current + 0.01Ω constantan wire, the output is:
  // 0.1V × 37.6 = 3.76V. Adding the OP offset base current (~0.385V), 
  // the total becomes:3.76V + 0.385V = 4.14V
  
  CM4M = 0;
  // CM4N from OPO
  
  // CM4P
  // VDD 5V*48/64 = 3.8V -> CM4RS = 48 
  //CM4REF = CM4REF_VDD | 48;
  
  // VDD 5V*53/64 = 4.14V -> CM4RS = 53 
  //CM4REF = CM4REF_VDD | 53;
  
  // VDD 5V*57/64 = 4.5V -> CM4RS = 57
  //CM4REF = CM4REF_VDD | 57;

  // VDD 5V*59/64 = 4.609V -> CM4RS = 59
  CM4REF = CM4REF_VDD | 59;
  
  // VDD 5V*61/64 = 4.8V -> CM4RS = 61 
  //CM4REF = CM4REF_VDD | 61; 
  
  CMDB4 = CM4_DEBOUNCE_8FCPU; // 0.5us
  CM4M = mskCM4SF | CM4_FALLING_TRIGGER | CM4N_OPO;
  IEN2 |= mskECMP4;
}


/**
 * @brief Adjust comparator 4 threshold for op-amp trim variation and enable.
 *
 * This function should compute the needed adjustment based on measured base current
 * vs. the default, then set CM4REF accordingly and enable CM4.
 */
void Surge_Protection_Modify(void)
{
  // Since the op-amp's trim value is reduced by 6 (due to die-to-die variations), 
  // the actual measured base_current must be compared with the default base_current 
  // to properly adjust the CMP4 protection threshold
  
  //TBD...
  
  // Enable CM4
  CM4M |= mskCM4EN;
}


/**
 * @brief ISR for comparator 0: PWM pulse trigger event.
 */
void comparator0_ISR(void) interrupt ISRCmp0
{
  if(pot_pulse_cnt < 250)
  {
    pot_pulse_cnt++;
  }    
  
}


/**
 * @brief ISR for comparator 1: handle AC overvoltage event.
 */
void comparator1_ISR(void) interrupt ISRCmp1
{  
  //Raise overvoltage flag
  ISR_f_Surge_Overvoltage_error  = 1;
  
  // In interrupt, simply stop the heating logic
  P01 = 1;  //PWM Pin
  PW0M = 0;
  PWM_INTERRUPT_DISABLE;
}


/**
 * @brief ISR for comparator 2: request PW0D decrease for HV protection.
 *
 * Marks a request flag instead of directly adjusting PW0D to avoid race conditions.
 * Raise CMP2 PW0D adjustment request flag.
 * Do NOT modify PW0D directly here to avoid race conditions with main/PWM0_ISR. (IC_BUG)
 * The actual PW0D adjustment will be handled safely by either PWM0_ISR or Power_Control().
 */
volatile bit PW0D_req_CMP2_isr  = 0;

void comparator2_ISR(void) interrupt ISRCmp2
{  
  // Only allow PW0D decrease for CMP2 HV protection when f_heating_initialized = 1,
  // Prevents wrong PW0D update during IGBT_C_SLOWDOWN or other PWM states.
  if (f_heating_initialized) {
    PW0D_req_CMP2_isr = TRUE;   // mark CMP2 request
    PW0F_CLEAR;
    PWM_INTERRUPT_ENABLE;
  }
  
}


/**
 * @brief ISR for comparator 3: AC zero-cross debounce and flag set.
 */
/*
   AC_SYNC_DEBOUNCE_TICKS is used to ensure that `ISR_f_CM3_AC_sync` is only triggered at the start of a new AC cycle:
   1. When `CM3+` is lower than `CM3-` (0.5V), `CM3_ISR` will be triggered, but there may be signal jitter.
   2. This variable sets `4ms (32 system_ticks)` as the debounce time.
   3. `ISR_f_CM3_AC_sync` is only allowed to be triggered again after `AC_SYNC_DEBOUNCE_TICKS`, preventing jitter from affecting the timing accuracy.
*/

#define AC_SYNC_DEBOUNCE_TICKS 32  // AC cycle sync debounce time  32*125us (4ms)

volatile bit ISR_f_CM3_AC_sync = 0;
volatile bit ISR_f_CM3_AC_Zero_sync = 0;
volatile bit ISR_f_CM3_AC_Periodic_sync = 0;
volatile uint8_t CM3_last_sync_tick = 0; // Record the time when `ISR_f_CM3_AC_sync` was set to `1`

void comparator3_ISR(void) interrupt ISRCmp3
{
  // Patch for T2    
  T2CH = 0x00;        // T2C clear
  T2CL = 0x00;
  P10 = ~P10; //HCW***
  
  // **Ensure that `ISR_f_CM3_AC_sync` is only triggered within a new AC cycle**
  if ((uint8_t)(system_ticks - CM3_last_sync_tick) >= AC_SYNC_DEBOUNCE_TICKS) {
    ISR_f_CM3_AC_sync = 1;
    ISR_f_CM3_AC_Zero_sync = 1;
    ISR_f_CM3_AC_Periodic_sync = 1;
    
    CM3_last_sync_tick = system_ticks; // **Update `last_sync_tick`**
  }
}


/**
 * @brief ISR for comparator 4: handle current overcurrent event.
 */
void comparator4_ISR(void) interrupt ISRCmp4
{  
  // Raise overcurrent flag
  ISR_f_Surge_Overcurrent_error  = 1;
  
  // In interrupt, simply stop the heating logic
  P01 = 1;  //PWM Pin
  PW0M = 0;
  PWM_INTERRUPT_DISABLE;
}
