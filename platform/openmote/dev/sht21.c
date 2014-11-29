/** \addtogroup openmote-cc2538
 * @{
 *
 * Driver for OpenMote-CC2538 humidity sensor
 * \file
 *  Temperature and relative humidity using a SHT21 sensor
 * \author
 *  Mehdi Migault
 */


#include <contiki.h>
#include "sht21.h"
#include "i2c.h"

/** \name Sensor's specific
 * @{
 */
#define SHT21_CRC_POLYNOMIAL		0x131
/** @} */
/*---------------------------------------------------------------------------*/
uint8_t
check_crc_SHT21(uint8_t data[], uint8_t dataSize, uint8_t chksm)
{
	uint8_t crc=0, i, j;
	for(i=0; i<dataSize; ++i) {
		crc ^= data[i];
		for(j=8; j>0; --j) {
			if(crc & 0x80) {
				crc = (crc<<1) ^ SHT21_CRC_POLYNOMIAL;
			} else {
				crc = (crc<<1);
			}
		}
	}
	if(crc != chksm) {
		return -1;
	} else {
		return I2C_MASTER_ERR_NONE;
	}
}
/*---------------------------------------------------------------------------*/
uint8_t
read_SHT21(uint16_t * data, uint8_t regist)
{
    uint16_t temp;
	uint8_t dataByte[2];
	if(regist != SHT21_TEMP_REGISTER && regist != SHT21_HUMI_REGISTER) {
		return -1;
	}

	i2c_master_set_slave_address(SHT21_SLAVE_ADDRESS, I2C_SEND);
	i2c_master_data_put(regist);
	i2c_master_command(I2C_MASTER_CMD_BURST_SEND_START);
	while(i2c_master_busy()){
	}
	if(i2c_master_error() == I2C_MASTER_ERR_NONE) {
		if(regist == SHT21_TEMP_REGISTER) {
			for(temp=0; temp<10; temp++) {
				clock_delay_usec(8500);	//85ms
			}
		} else if(regist == SHT21_HUMI_REGISTER) {
			for(temp=0; temp<10; temp++) {
				clock_delay_usec(2900);	//29ms
			}
		}
		/* Get the 2 bytes of data*/
		/* Data MSB */
		i2c_master_set_slave_address(SHT21_SLAVE_ADDRESS, I2C_RECEIVE);
		i2c_master_command(I2C_MASTER_CMD_BURST_RECEIVE_START);
		while(i2c_master_busy()) {
		}
		if(i2c_master_error() == I2C_MASTER_ERR_NONE) {
			*data = i2c_master_data_get() << 8;
			
			/* Data LSB */
			i2c_master_command(I2C_MASTER_CMD_BURST_RECEIVE_CONT);
			while(i2c_master_busy()) {
			}
			if(i2c_master_error() == I2C_MASTER_ERR_NONE) {
				*data |= i2c_master_data_get();
				
				/* Checksum */
				i2c_master_command(I2C_MASTER_CMD_BURST_RECEIVE_FINISH);
				while(i2c_master_busy()) {
				}
				if(i2c_master_error() == I2C_MASTER_ERR_NONE) {
					dataByte[0] = (*data)>>8;
					dataByte[1] = (*data)&0x00FF;
					if(check_crc_SHT21(dataByte, 2, i2c_master_data_get()) == I2C_MASTER_ERR_NONE){
						return I2C_MASTER_ERR_NONE;
					}
				}
			}
		}
	}
	return i2c_master_error();
}
/*---------------------------------------------------------------------------*/
/**
 * @}
 */
