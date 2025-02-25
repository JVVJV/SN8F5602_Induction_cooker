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
#include "I2C.h"

/*_____ D E F I N I T I O N S ______________________________________________*/
#define SLAVE_ADDRESS  0x55  // **Slave 位址**

#define I2C_WRITE_MODE  0
#define I2C_READ_MODE   1



/*_____ D E C L A R A T I O N S ____________________________________________*/
volatile uint8_t* I2C_Buf; 
uint8_t I2C_Slave_Address;
uint8_t I2C_Length;
volatile uint8_t I2C_Index;
volatile bit I2C_Mode;
volatile bit f_i2c_power_received = 0;   // **功率接收完成標誌**

/*_____ M A C R O S ________________________________________________________*/

/*_____ F U N C T I O N S __________________________________________________*/
void I2C_Init(void)
{
  I2CCON = 0;
  
  /*	Configure the I2C's IO status and register */
  I2C_PIN_SET_INPUT_1;
  SMBSEL = mskI2CMX;
  
  // I2C 32MHz/320 = 100KHz
  I2CCON = mskI2CCON_I2C_ENABLE | mskI2CCON_I2C_DIV_320;
  
  /*	Enable I2C interrupt  */
  IEN1 = mskIEN1_INT_I2C;
}

uint8_t I2C_Calculate_Checksum(uint8_t *databuf, uint8_t length) {
    uint8_t i, sum = 0;
    for (i = 0; i < length; i++) {
        sum += databuf[i];
    }
    return ~sum; // **反向總和**
}


void I2C_Write(uint8_t slave_addr, uint8_t *databuf, uint8_t length) {
    // **計算並附加 `Checksum`**
    databuf[length] = I2C_Calculate_Checksum(databuf, length);
    length++;  // **包含 `Checksum`**

    // **設定 I2C 傳輸參數**
    I2C_Slave_Address = slave_addr;
    I2C_Buf = databuf;
    I2C_Length = length;
    I2C_Index = 0;
    I2C_Mode = I2C_WRITE_MODE;

    // **開始 I2C 傳輸**
    I2CCON |= mskI2CCON_I2C_STA; // **啟動 I2C**
}
void I2C_Read(uint8_t slave_addr, uint8_t *databuf, uint8_t length) {
    // **設定 I2C 讀取參數**
    I2C_Slave_Address = slave_addr;
    I2C_Buf = databuf;
    I2C_Length = length;
    I2C_Index = 0;
    I2C_Mode = I2C_READ_MODE;

    // **開始 I2C 讀取**
    I2CCON |= mskI2CCON_I2C_STA; // **啟動 I2C**
}

void I2C_ISR(void) interrupt ISRI2c {
    switch (I2CSTA) {
        case 0x08: // **Start Condition Transmitted**
        case 0x10: // **Repeated Start**
            I2CDAT = (I2C_Slave_Address << 1) | I2C_Mode;
            break;

        case 0x18: // **Address ACKed (TX Mode)**
        case 0x28: // **Data ACKed (TX Mode)**
            I2CCON &= ~mskI2CCON_I2C_STA;
        
            if (I2C_Index < I2C_Length) {
              I2CDAT = I2C_Buf[I2C_Index++];
            } else {
              I2CCON &= ~mskI2CCON_I2C_STA;
              I2CCON |= mskI2CCON_I2C_STO;  // **發送 Stop**
              I2C_Index = 0;
            }
            break;

        case 0x20: // **Address NACK (TX Mode)**
        case 0x30: // **Data NACK (TX Mode)**
            I2CCON &= ~mskI2CCON_I2C_STA;
            I2CCON |= mskI2CCON_I2C_STO;  // **發送 Stop**
            I2C_Index = 0;
            break;

        case 0x40: // **Address ACKed (RX Mode)**
            I2CCON &= ~mskI2CCON_I2C_STA;
        
            if (I2C_Index < I2C_Length - 1) {
                I2CCON |= mskI2CCON_I2C_AA;  // **繼續接收**
            } else {
                I2CCON &= ~mskI2CCON_I2C_AA; // **最後一個字節**
            }
            break;
        
        case 0x48:      /*  after addr_NACK, repeat start condition */
            I2CCON |= mskI2CCON_I2C_STA;
            I2CCON |= mskI2CCON_I2C_STO;
            break;
        case 0x50: // **Data ACKed (RX Mode)**
            I2C_Buf[I2C_Index++] = I2CDAT;
            if (I2C_Index < I2C_Length - 1) {
                I2CCON |= mskI2CCON_I2C_AA;  // **繼續接收**
            } else {
                I2CCON &= ~mskI2CCON_I2C_AA; // **最後一個字節**
            }
            break;

        case 0x58: // **Data NACKed (RX Mode)**
            I2C_Buf[I2C_Index++] = I2CDAT;
            I2CCON &= ~mskI2CCON_I2C_AA;
            I2CCON |= mskI2CCON_I2C_STO;  // **發送 Stop**
            I2C_Index = 0;
            // **驗證 Checksum**
            if (I2C_Buf[1] == ~(I2C_Buf[0])) {  
                f_i2c_power_received = 1;  // **標記功率接收完成**
            }
            break;

        case 0xF8: // **Bus Error**
            I2CCON |= mskI2CCON_I2C_STO;  // **發送 Stop**
            I2C_Index = 0;
            break;
        
        default:				
            I2CCON &= ~mskI2CCON_I2C_STA;
            I2CCON |= mskI2CCON_I2C_STO;
            break;
    }

    // **清除 I2C 旗標**
    I2CCON &= ~mskI2CCON_I2C_FLAG;
}

