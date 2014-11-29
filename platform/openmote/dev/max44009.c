/** \addtogroup openmote-cc2538
 * @{
 *
 * Driver for OpenMote-CC2538 light sensor
 * \file
 *  Light sensor - max44009
 * \author
 *  OpenMote users
 */


#include <contiki.h>
#include "max44009.h"
#include "i2c.h"

/** \name Sensor's specific
 * @{
 *
 *================================ define ===================================*/
/* ADDRESS AND NOT_FOUND VALUE */
#define MAX44009_SLAVE_ADDRESS              ( 0x4A )
#define MAX44009_NOT_FOUND                  ( 0x00 )

/* REGISTER ADDRESSES */
#define MAX44009_INT_STATUS_ADDR            ( 0x00 )    // R
#define MAX44009_INT_ENABLE_ADDR            ( 0x01 )    // R/W
#define MAX44009_CONFIG_ADDR                ( 0x02 )    // R/W
#define MAX44009_LUX_HIGH_ADDR              ( 0x03 )    // R
#define MAX44009_LUX_LOW_ADDR               ( 0x04 )    // R
#define MAX44009_THR_HIGH_ADDR              ( 0x05 )    // R/W
#define MAX44009_THR_LOW_ADDR               ( 0x06 )    // R/W
#define MAX44009_THR_TIMER_ADDR             ( 0x07 )    // R/W

/* INTERRUPT VALUES */
#define MAX44009_INT_STATUS_OFF             ( 0x00 )
#define MAX44009_INT_STATUS_ON              ( 0x01 )
#define MAX44009_INT_DISABLED               ( 0x00 )
#define MAX44009_INT_ENABLED                ( 0x01 )

/* CONFIGURATION VALUES */
#define MAX44009_CONFIG_DEFAULT             ( 0 << 7 )
#define MAX44009_CONFIG_CONTINUOUS          ( 1 << 7 )
#define MAX44009_CONFIG_AUTO                ( 0 << 6 )
#define MAX44009_CONFIG_MANUAL              ( 1 << 6 )
#define MAX44009_CONFIG_CDR_NORMAL          ( 0 << 5 )
#define MAX44009_CONFIG_CDR_DIVIDED         ( 1 << 5 )
#define MAX44009_CONFIG_INTEGRATION_800ms   ( 0 << 0 )
#define MAX44009_CONFIG_INTEGRATION_400ms   ( 1 << 0 )
#define MAX44009_CONFIG_INTEGRATION_200ms   ( 2 << 0 )
#define MAX44009_CONFIG_INTEGRATION_100ms   ( 3 << 0 )
#define MAX44009_CONFIG_INTEGRATION_50ms    ( 4 << 0 )
#define MAX44009_CONFIG_INTEGRATION_25ms    ( 5 << 0 )
#define MAX44009_CONFIG_INTEGRATION_12ms    ( 6 << 0 )
#define MAX44009_CONFIG_INTEGRATION_6ms     ( 7 << 0 )

/* DEFAULT CONFIGURATION */
#define MAX44009_DEFAULT_CONFIGURATION      ( MAX44009_CONFIG_DEFAULT | \
                                              MAX44009_CONFIG_AUTO | \
                                              MAX44009_CONFIG_CDR_NORMAL | \
                                              MAX44009_CONFIG_INTEGRATION_100ms )


uint8_t exponent;
uint8_t mantissa;

/** @} */
/*---------------------------------------------------------------------------*/
int enable_max44009(void)
{
    uint8_t max44009_address[5] = {MAX44009_INT_ENABLE_ADDR, MAX44009_CONFIG_ADDR, \
                                   MAX44009_THR_HIGH_ADDR, MAX44009_THR_LOW_ADDR, \
                                   MAX44009_THR_TIMER_ADDR};
    uint8_t max44009_value[5];
    uint8_t status;

    max44009_value[0] = (MAX44009_INT_STATUS_ON);
    max44009_value[1] = (MAX44009_DEFAULT_CONFIGURATION);
    max44009_value[2] = (0xFF);
    max44009_value[3] = (0x00);
    max44009_value[4] = (0xFF);

    
    for(uint8_t i=0; i<sizeof(max44009_address); i++)
    {

        i2c_master_set_slave_address(MAX44009_ADDRESS, I2C_SEND);

        i2c_master_data_put(max44009_address[i]);  // send address 
        i2c_master_command(I2C_MASTER_CMD_BURST_SEND_START);
        while(i2c_master_busy())   {}
        if(I2C_MASTER_ERR_NONE != i2c_master_error() ){
            return -1;
        }

        i2c_master_data_put(max44009_value[i]);    // send config data
        i2c_master_command(I2C_MASTER_CMD_BURST_SEND_STOP);
        while(i2c_master_busy())   {}
        if(I2C_MASTER_ERR_NONE != i2c_master_error() ){
            return -1;
        }
    }

    return 0;
}
/** @} */
/*---------------------------------------------------------------------------*/
int rst_max44009(void)
{
    uint8_t max44009_address[5] = {MAX44009_INT_ENABLE_ADDR, MAX44009_CONFIG_ADDR, \
                                   MAX44009_THR_HIGH_ADDR, MAX44009_THR_LOW_ADDR, \
                                   MAX44009_THR_TIMER_ADDR};
    uint8_t max44009_value[5];
    uint8_t status;

    max44009_value[0] = (MAX44009_INT_STATUS_ON);
    max44009_value[1] = (MAX44009_DEFAULT_CONFIGURATION);
    max44009_value[2] = (0x00);
    max44009_value[3] = (0x03);
    max44009_value[4] = (0xFF);
    max44009_value[4] = (0x00);
    max44009_value[4] = (0xFF);

    
    for(uint8_t i=0; i<sizeof(max44009_address); i++)
    {

        i2c_master_set_slave_address(MAX44009_SLAVE_ADDRESS, I2C_SEND);

        i2c_master_data_put(max44009_address[i]);  // send address 
        i2c_master_command(I2C_MASTER_CMD_BURST_SEND_START);
        while(i2c_master_busy())   {}
        if(I2C_MASTER_ERR_NONE != i2c_master_error() ){
            return -1;
        }

        i2c_master_data_put(max44009_value[i]);    // send config data
        i2c_master_command(I2C_MASTER_CMD_BURST_SEND_STOP);
        while(i2c_master_busy())   {}
        if(I2C_MASTER_ERR_NONE != i2c_master_error() ){
            return -1;
        }
    }

    return 0;
}



int readReg(uint8_t addr, uint8_t *data)
{
    uint8_t max44009_data[2];

    i2c_master_set_slave_address(MAX44009_SLAVE_ADDRESS, I2C_SEND);
    i2c_master_data_put(addr);
    i2c_master_command(I2C_MASTER_CMD_BURST_SEND_START);
    while(i2c_master_busy())   {}

    if(I2C_MASTER_ERR_NONE != i2c_master_error()){
        return -1;
    }

    i2c_master_set_slave_address(MAX44009_SLAVE_ADDRESS, I2C_RECEIVE);
    i2c_master_command(I2C_MASTER_CMD_SINGLE_RECEIVE);
    while(i2c_master_busy())  {}

    if(I2C_MASTER_ERR_NONE != i2c_master_error()){
        return -1;
    }

    *data = i2c_master_data_get();
    return 0;
}

int readLux(void)
{
    uint8_t regContent[2];
    int stat;

    i2c_master_data_put();
    stat = readReg(MAX44009_LUX_HIGH_ADDR, &regContent[0]);
    if(stat < 0){
        return -1;
    }

	clock_delay_usec(10);	//10us

    stat = readReg(MAX44009_LUX_LOW_ADDR, &regContent[1]);
    if(stat < 0){
        return -1;
    }

    exponent = (( regContent[0] >> 4) & 0x0E);
    mantissa = (( regContent[0] & 0x0F) << 4) | (regContent[1] & 0x0F);

    return 0;
}

float getLux(void)
{
    float lux = 0.045;
    lux *= 2^exponent * mantissa;

    return lux;
}
/*---------------------------------------------------------------------------*/
