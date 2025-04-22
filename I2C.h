#ifndef __I2C_H__
#define __I2C_H__

/*_____ I N C L U D E S ____________________________________________________*/
#include <SN8F5602.h>

/*_____ D E F I N I T I O N S ______________________________________________*/
#define SLAVE_SEL                 0
#define MASTER_SEL                1

#define I2C_R                     1
#define I2C_W                     0

#define	mskI2CCON_I2C_DIV_40			(0x00)
#define	mskI2CCON_I2C_DIV_80			(0x01)
#define	mskI2CCON_I2C_DIV_160			(0x02)
#define	mskI2CCON_I2C_DIV_320			(0x03)

#define	mskI2CCON_I2C_AA								(0x01U<<2)
#define	mskI2CCON_I2C_FLAG							(0x01U<<3)
#define	mskI2CCON_I2C_STO								(0x01U<<4)
#define	mskI2CCON_I2C_STA								(0x01U<<5)
#define	mskI2CCON_I2C_ENABLE						(0x01U<<6)
#define	mskI2CCON_I2C_CLOCKDIV_T1OV			(0x01U<<7)

#define	mskI2CADR_I2C_GENERALCALL				(0x01U<<0)

#define mskI2CDAT_I2C_MODE_TX						(0x00U<<0)
#define mskI2CDAT_I2C_MODE_RX						(0x01U<<0)

#define	mskIEN1_INT_I2C									(0x01U<<0)


#define	mskI2CMX    (1<<3)


#define I2C_BUFFER_SIZE  3  // **最大緩衝區長度**

/*_____ D E C L A R A T I O N S ____________________________________________*/
extern volatile bit f_i2c_power_received;   // **功率接收完成標誌**

/*_____ M A C R O S ________________________________________________________*/
#define I2C_PIN_SET_INPUT_0     P0M &= ~((0x01U<<6)|(0x01U<<7));  //SCL = P06,SDA = P07.
#define I2C_PIN_SET_INPUT_1     P1M &= ~((0x01U<<3)|(0x01U<<4));	//SCL = P13,SDA = P14.

/*_____ F U N C T I O N S __________________________________________________*/
void I2C_Init(void);
void I2C_Write(uint8_t slave_addr, uint8_t *databuf, uint8_t length);
void I2C_Read(uint8_t slave_addr, uint8_t *databuf, uint8_t length);

#endif  // __I2C_H__
