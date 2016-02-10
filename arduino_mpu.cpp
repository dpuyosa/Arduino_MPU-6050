#include "arduino_mpu.h"
#include <I2Cdev.h>

#ifdef __cplusplus
extern "C" {
#endif

	int8_t i2c_write(unsigned char slave_addr, unsigned char reg_addr,unsigned char length, unsigned char const *data) {
		return !I2Cdev::writeBytes(slave_addr,reg_addr,length,(uint8_t *)data);
	}

	int8_t i2c_read(unsigned char slave_addr, unsigned char reg_addr,unsigned char length, unsigned char *data) {
		return !I2Cdev::readBytes(slave_addr,reg_addr,length,(uint8_t *)data);
	}

#ifdef __cplusplus
}
#endif