#include <I2Cdev.h>
#include <arduino_mpu.h>
#include <inv_mpu.h>
//#include <inv_mpu_dmp_motion_driver.h>

const float Q16 = 65536.0;

float gyro_sens;
unsigned short accel_sens;
long temp;
void setup() {
  Serial.begin(115200);
  while(!Serial);
  Serial.println(mpu_init(NULL));
  Serial.println(mpu_set_sensors(INV_XYZ_GYRO|INV_XYZ_ACCEL));
  Serial.println(mpu_configure_fifo(INV_XYZ_GYRO|INV_XYZ_ACCEL));
  //Serial.println(dmp_load_motion_driver_firmware());
  //Serial.println(mpu_set_dmp_state(1));
  //Serial.println(dmp_enable_feature(DMP_FEATURE_SEND_RAW_ACCEL|DMP_FEATURE_SEND_CAL_GYRO|DMP_FEATURE_GYRO_CAL));
  
  mpu_get_gyro_sens(&gyro_sens);
  mpu_get_accel_sens(&accel_sens);
}

short gyro[3];
short accel[3];
//unsigned char more;
//short sensors;
//int res;
void loop() {
  delay(800);
  mpu_get_temperature(&temp, NULL);
  mpu_get_gyro_reg(gyro, NULL);
  mpu_get_accel_reg(accel, NULL);
 /* do {
    res = dmp_read_fifo(gyro,accel,NULL,&sensors,&more);
    Serial.print(res);
  } while (more>1);*/
  Serial.println();
  Serial.print("Temp: ");Serial.print(temp/Q16);Serial.println("C");
  Serial.print("Gyro: ");Serial.print(gyro[0]/gyro_sens);Serial.print(" ");Serial.print(gyro[1]/gyro_sens);Serial.print(" ");Serial.println(gyro[2]/gyro_sens);
  Serial.print("Acce: ");Serial.print(accel[0]/(float)accel_sens);Serial.print(" ");Serial.print(accel[1]/(float)accel_sens);Serial.print(" ");Serial.println(accel[2]/(float)accel_sens);
  Serial.println();
}
