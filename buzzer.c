/**
 * @file buzzer.c
 * @brief Buzzer and fan control functions.
 * 
 * Provides initialization for buzzer PWM output and
 * dynamic fan speed control (normal vs. full speed) based
 * on IGBT heat warning status.
 *
 * c 2024 SONiX
 */

/*_____ I N C L U D E S ____________________________________________________*/
#include "buzzer.h"
#include "system.h"

/*_____ D E C L A R A T I O N S ____________________________________________*/

/*_____ D E F I N I T I O N S ______________________________________________*/

/*_____ M A C R O S ________________________________________________________*/

/*_____ F U N C T I O N S __________________________________________________*/
/**
 * @brief Initialize buzzer PWM to generate 500Hz tone.
 *
 * Configures the buzzer generator to a 500Hz output
 * using a 32MHz clock divided by 65536.
 */
void Buzzer_Init(void)
{
  BZM = 0;
  
  // 32MHz/65536 = 500Hz
  BZM = BZ_RATE_65536;
}


/**
 * @brief Enable fan speed based on heat warning flag.
 *
 * If IGBT heat warning is active, switch to full speed;
 * otherwise use normal PWM-driven speed.
 */
void Fan_Enable(void)
{
  // if we¡¦re in a heat warning, go full; otherwise normal
  if (warning_flags.f.IGBT_heat_warning) {
    Fan_SetFullSpeed();
  } else {
    Fan_SetNormalSpeed();
  }
}


/**
 * @brief Set fan to normal (PWM-driven) speed.
 *
 * Enables the buzzer PWM output used to drive the fan at
 * standard speed and disables the full-speed GPIO output on P0.0.
 */
void Fan_SetNormalSpeed(void)
{
  // Enable PWM-driven (buzzer) output for normal fan speed
  BUZZER_ENABLE;  
  
  // Disable the GPIO full-speed drive on P0.0
  // (return P0.0 to its normal quasi-bidirectional/PWM function)
  P0M &= ~(1 << 0);
}


/**
 * @brief Set fan to full speed via GPIO drive.
 *
 * Configures P0.0 as a push-pull GPIO output and drives it
 * high to achieve maximum fan speed. Disables PWM-driven output.
 */
void Fan_SetFullSpeed(void)
{
  // Configure P0.0 as GPIO output
  P0M |= (1 << 0);
  // Drive P0.0 high for full-on fan speed
  P00 = 1;

  // Disable PWM-driven (buzzer) output
  BUZZER_DISABLE;
}

