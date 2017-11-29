/*
 * file: i2c.c
 *
 * PowerPC 8240  partial I2C support - enough to test external interrupts
 * for UCOS-II
 *
 * Author: Ernie Price
 * eprice@bitwise.net
 *
 */
#include "includes.h"

volatile struct i2c
{
   INT8U  i2cadr;
   INT8U  dum1[3];
   INT8U  i2cfdr;
   INT8U  dum2[3];
   INT8U  i2ccr;
        #define  I2CCR_MEN      0x80    // master enable
        #define  I2CCR_MIEN     0x40    // master interrupt enable
        #define  I2CCR_MSTA     0x20    // master status
        #define  I2CCR_MTX      0x10    // master transmit
        #define  I2CCR_TXAK     0x08    // transmit ACK
        #define  I2CCR_RSTA     0x04    // 

   INT8U  dum3[3];
   INT8U  i2csr;
        #define  I2CSR_MCF      0x80
        #define  I2CSR_MAAS     0x40
        #define  I2CSR_MBB      0x20
        #define  I2CSR_MAL      0x10
        #define  I2CSR_SRW      0x04
        #define  I2CSR_MIF      0x02
        #define  I2CSR_RXAK     0x01
        
   INT8U  dum4[3];
   INT8U  i2cdr;
} *i2cCtrl = (struct i2c*)(EUMB_BASE + 0x3000);

/*
    \brief Processes an i2c interrupt
*/
static void i2c_Handler(INT32U arg)
{
#if 0
   URP(arg);
   i2cCtrl->i2csr &= ~I2CSR_MIF;
#endif

}

/*
    \brief Initializes the I2C unit
*/
void i2c_Init(void)     
{
#if 0
    i2cCtrl->i2cfdr = 0x22;                  // set I2C clock frequency
   
    /*
        The following block of code seems to reliably get the 8240 I2C controller
        out of the hung condition that can occur if one of the slave devices hangs
        such that it is driving the SDA line.
    */
    i2cCtrl->i2ccr = I2CCR_MEN  | I2CCR_MSTA;
    i2cCtrl->i2cdr = 0;                         // some data
    DelayNmsec(1);
    i2cCtrl->i2ccr = I2CCR_MSTA | I2CCR_TXAK | I2CCR_RSTA;
    DelayNmsec(1);
    i2cCtrl->i2ccr = I2CCR_MEN  | I2CCR_MSTA | I2CCR_TXAK | I2CCR_RSTA;
    i2cCtrl->i2cdr = 0;                         // some data
    DelayNmsec(1);
    /*
        End recovery code
    */

    DelayNusec(25);               // prevent back-to-back problems
    epic_AddHandler(I2C, i2c_Handler, 0);

    return; // I2CSUCCESS;
#endif
}

/*
    \brief Enables the I2C unit
*/
void i2c_Enable(void)
{
#if 0
    i2cCtrl->i2ccr = I2CCR_MEN | I2CCR_MIEN | I2CCR_MSTA | I2CCR_MTX;
#endif
}

/*
    \brief Sends a INT8Uacter on the I2C bus
*/
void i2c_SendChar(INT8U kh)
{
#if 0
    i2cCtrl->i2cdr = kh;
    IRQLedManipulate(0x80);          // show that interrupt is working
#endif
}

/* End of Source */
