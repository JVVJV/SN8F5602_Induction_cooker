/******************** (C) COPYRIGHT 2025 SONiX *******************************
 * @file    I2C.c
 * @brief   I2C driver implementation for SN8F5602 microcontroller.
 * @author  HCW
 * @date    2025/01
 * @company SONiX Technology Co., Ltd.
 *****************************************************************************/
 
/*_____ I N C L U D E S ____________________________________________________*/
#include "I2C.h"

/*_____ D E F I N I T I O N S ______________________________________________*/
#define SLAVE_ADDRESS  0x55

#define I2C_WRITE_MODE  0
#define I2C_READ_MODE   1

/*_____ D E C L A R A T I O N S ____________________________________________*/
volatile uint8_t* I2C_Buf; 
uint8_t I2C_Slave_Address;
uint8_t I2C_Length;

volatile bit I2C_Mode;
volatile uint8_t I2C_Index;                 // Index into I2C_Buf for current transfer
volatile bit ISR_f_i2c_power_received = 0;  // power data received
volatile bit ISR_f_I2C_error = 0;

/*_____ M A C R O S ________________________________________________________*/

/*_____ F U N C T I O N S __________________________________________________*/
/**
 * @brief  Initialize I2C peripheral as SMBus mode at 100kHz.
 * @note   Configures pins, selects master mode and enables the I2C interrupt.
 */
void I2C_Init(void)
{
  I2CCON = 0;
  
  /*	Configure the I2C's IO status and register */
  I2C_PIN_SET_INPUT_1;
  SMBSEL = mskI2CMX;
  
  // I2C 32MHz/320 = 100KHz
  I2CCON = mskI2CCON_I2C_ENABLE | mskI2CCON_I2C_DIV_320;
  
  /*	Enable I2C interrupt  */
  IEN1 |= mskIEN1_INT_I2C;
}


/**
 * @brief  Compute 8-bit checksum by summing all bytes and bitwise inverting.
 * @param  databuf Pointer to data buffer to checksum.
 * @param  length  Number of bytes to include in checksum.
 * @return Inverted 8-bit sum of all bytes.
 */
uint8_t I2C_Calculate_Checksum(uint8_t *databuf, uint8_t length) {
    uint8_t i, sum = 0;
    for (i = 0; i < length; i++) {
        sum += databuf[i];
    }
    return ~sum;
}


/**
 * @brief  Start an I2C write transaction, appending checksum.
 * @param  slave_addr I2C slave address.
 * @param  databuf    Pointer to buffer containing data to send.
 * @param  length     Number of data bytes to send (excluding checksum).
 */
void I2C_Write(uint8_t slave_addr, uint8_t *databuf, uint8_t length) {
    // Calculate and append checksum
    databuf[length] = I2C_Calculate_Checksum(databuf, length);
    length++;

    /* Set up transfer parameters */
    I2C_Slave_Address = slave_addr;
    I2C_Buf = databuf;
    I2C_Length = length;
    I2C_Index = 0;
    I2C_Mode = I2C_WRITE_MODE;

    /* Generate START condition */
    I2CCON |= mskI2CCON_I2C_STA; // **啟動 I2C**
}

/**
 * @brief  Start an I2C read transaction.
 * @param  slave_addr I2C slave address.
 * @param  databuf    Pointer to buffer to receive data.
 * @param  length     Number of bytes to read.
 */
void I2C_Read(uint8_t slave_addr, uint8_t *databuf, uint8_t length) {
    /* Set up transfer parameters */
    I2C_Slave_Address = slave_addr;
    I2C_Buf = databuf;
    I2C_Length = length;
    I2C_Index = 0;
    I2C_Mode = I2C_READ_MODE;

    /* Generate START condition */
    I2CCON |= mskI2CCON_I2C_STA; // **啟動 I2C**
}


/**
 * @brief  I2C interrupt service routine handles all states.
 */
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
                ISR_f_i2c_power_received = 1; // power received
            } else {
                ISR_f_I2C_error = 1;          // checksum error
            }
            break;

        case 0xF8: // **Bus Error**
        default:
            I2C_Index = 0;
            I2CCON &= ~mskI2CCON_I2C_STA;
            I2CCON |= mskI2CCON_I2C_STO;
            
            I2CCON &= ~mskI2CCON_I2C_ENABLE; //HCW***
            I2CCON |= mskI2CCON_I2C_ENABLE;
            ISR_f_I2C_error = 1;            // bus error
            break;
    }

    // **清除 I2C 旗標**
    I2CCON &= ~mskI2CCON_I2C_FLAG;
}

