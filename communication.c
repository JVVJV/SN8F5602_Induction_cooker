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

/*_____ D E F I N I T I O N S ______________________________________________*/


/*_____ D E C L A R A T I O N S ____________________________________________*/
uint8_t i2c_status_code = I2C_STATUS_NORMAL;  // default normal status

/*_____ M A C R O S ________________________________________________________*/

/*_____ F U N C T I O N S __________________________________________________*/
void I2C_Communication(void) {
    static bit f_i2c_transaction_type = 1; // **0: Read, 1: Write**
    static uint8_t i2c_buffer[I2C_BUFFER_SIZE];

    if (cntdown_timer_expired(CNTDOWN_TIMER_I2C)) {
      if (f_i2c_power_received) {
        f_i2c_power_received = 0;
        switch (i2c_buffer[0]) {
          case 0x00: power_setting = 0; break;
          case 0x44: power_setting = 200000; break;
          case 0x48: power_setting = 500000; break;
          case 0x49: power_setting = 800000; break;
          case 0x4A: power_setting = 1000000; break;
          case 0x4B: power_setting = 1300000; break;
          case 0x4C: power_setting = 1600000; break;
          case 0x4D: power_setting = 1800000; break;
          case 0x4E: power_setting = 2000000; break;
          case 0x4F: power_setting = 2200000; break;
          default:   power_setting = 0;       break; // **�p�G�ƾڵL��
        } 
      }        
      
      if (f_i2c_transaction_type) {
            // **�ǳ� NTC ���**
            i2c_buffer[0] = i2c_status_code;
            i2c_buffer[1] = (uint8_t)TOP_TEMP_C;
            I2C_Write(I2C_SLAVE_ADDRESS, i2c_buffer, 2);
        } else {
            I2C_Read(I2C_SLAVE_ADDRESS, i2c_buffer, 2); // **Ū���\�v**
        }

        f_i2c_transaction_type = !f_i2c_transaction_type; // **���Ū/�g**
        

        // **���s�Ұ� 43ms �p��**
        cntdown_timer_start(CNTDOWN_TIMER_I2C, I2C_INTERVAL);
    }
}

