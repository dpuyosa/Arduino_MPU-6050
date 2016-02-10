 /* The following functions must be defined for this platform:
 * i2c_write(unsigned char slave_addr, unsigned char reg_addr,
 *      unsigned char length, unsigned char const *data)
 * i2c_read(unsigned char slave_addr, unsigned char reg_addr,
 *      unsigned char length, unsigned char *data)
 * delay_ms(unsigned long num_ms)
 * get_ms(unsigned long *count)
 * reg_int_cb(void (*cb)(void), unsigned char port, unsigned char interrupt)
 * labs(long x)
 * fabsf(float x)
 * min(int a, int b)
 */
#ifndef __ARDUINO_MPU__
#define __ARDUINO_MPU__
 
#define MPU6050
#define MPU_EXTRAS //Enable extras: tap, shake, pedometer and android orient
#include <Arduino.h>

#define delay_ms delay
#define log_i(...) do {} while (0)
#define log_e(...) do {} while (0)
//#define min(a,b) ((a<b) ? a : b)
//#define labs(x) ((x>0) ? x : -x)
//#define fabsf(x) labs(x)

struct int_param_s {
    uint8_t pin;
    void (*cb)(void);
};

static inline void get_ms(unsigned long *count){
	*count = millis();
}

static inline void reg_int_cb(struct int_param_s *int_param){
	attachInterrupt(digitalPinToInterrupt(int_param->pin), int_param->cb, RISING);
}
 
#ifdef __cplusplus
extern "C" {
#endif

	int8_t i2c_write(unsigned char slave_addr, unsigned char reg_addr,unsigned char length, unsigned char const *data);
	int8_t i2c_read(unsigned char slave_addr, unsigned char reg_addr,unsigned char length, unsigned char *data);

#ifdef __cplusplus
}
#endif

#endif // __ARDUINO_MPU__