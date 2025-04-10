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
#include <SN8F5602.h>


/*_____ D E F I N I T I O N S ______________________________________________*/
#define I2C_STATUS_NORMAL           0x70  // Device normal
#define I2C_STATUS_OVERVOLTAGE      0x71  // Surge: Over voltage
#define I2C_STATUS_OVERCURRENT      0x72  // Surge: Over current
#define I2C_STATUS_OTHER_ERROR      0x73  // Other error flag active
#define I2C_STATUS_IGBT_OVERHEAT    0x74  // 

/*_____ D E C L A R A T I O N S ____________________________________________*/
extern uint8_t i2c_status_code;  // default normal status

/*_____ M A C R O S ________________________________________________________*/

/*_____ F U N C T I O N S __________________________________________________*/
void I2C_Communication(void);


