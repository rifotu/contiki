
/** \name Sensor's specific
 * @{
 */
#define SHT21_SLAVE_ADDRESS			0x40
#define SHT21_TEMP_REGISTER			0xF3
#define SHT21_HUMI_REGISTER			0xF5
#define SHT21_DECIMAL_PRECISION		   2
/** @} */


uint8_t read_SHT21(uint16_t * data, uint8_t regist);

