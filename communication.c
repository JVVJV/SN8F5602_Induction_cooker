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
#include "communication.h"
#include "I2C.h"
#include "system.h"
#include "temperature.h"
#include "config.h"
#include "power.h"

/*_____ D E F I N I T I O N S ______________________________________________*/


/*_____ D E C L A R A T I O N S ____________________________________________*/
uint8_t i2c_status_code = I2C_STATUS_NORMAL;  // default normal status

/*_____ M A C R O S ________________________________________________________*/

/*_____ F U N C T I O N S __________________________________________________*/
void I2C_Communication(void) {
    static bit f_i2c_transaction_type = 1; // **0: Read, 1: Write**
    static uint8_t i2c_buffer[I2C_BUFFER_SIZE];
    static uint8_t i2c_comm_fail_count = 0; 

    if (cntdown_timer_expired(CNTDOWN_TIMER_I2C)) {
      // Restart 43 ms countdown
      cntdown_timer_start(CNTDOWN_TIMER_I2C, I2C_INTERVAL);
      
      if (ISR_f_i2c_power_received) {
        ISR_f_i2c_power_received = 0;
        i2c_comm_fail_count = 0;  // Reset failure counter
        
        switch (i2c_buffer[0]) {
          case 0x00: power_level = 0; break;  // 0 W
          case 0x44: power_level = 1; break;  // 200 W
          case 0x48: power_level = 2; break;  // 500 W
          case 0x49: power_level = 3; break;  // 800 W
          case 0x4A: power_level = 4; break;  // 1000 W
          case 0x4B: power_level = 5; break;  // 1300 W
          case 0x4C: power_level = 6; break;  // 1600 W
          case 0x4D: power_level = 7; break;  // 1800 W
          case 0x4E: power_level = 8; break;  // 2000 W
          case 0x4F: power_level = 9; break;  // Not support 2.2K WHCW***
          default:   power_level = 0; break;  // Invalid data
        }
      } else {
        // No response: count failures and reinit after 6
        if (++i2c_comm_fail_count >= 6) {
            i2c_comm_fail_count = 0;
            I2C_Init();
        }
      }
      
      // Alternate between write (NTC) and read (power)
      if (f_i2c_transaction_type) {
          i2c_buffer[0] = i2c_status_code;
          i2c_buffer[1] = (uint8_t)TOP_TEMP_C;
          I2C_Write(I2C_SLAVE_ADDRESS, i2c_buffer, 2);
      } else {
          I2C_Read(I2C_SLAVE_ADDRESS, i2c_buffer, 2);
      }

      f_i2c_transaction_type = !f_i2c_transaction_type;
    }
}

