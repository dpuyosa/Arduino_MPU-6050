#include <I2Cdev.h>
#include <arduino_mpu.h>
#include <inv_mpu.h>

volatile bool new_int = false;
void interrupt(){
  new_int = true;
}

float gyro_sens;
unsigned short accel_sens;
void setup() {
  uint8_t errors = 0;
  Serial.begin(115200);
  while(!Serial);
  Serial.println("Setting up");
  struct int_param_s params = { 7, interrupt };
  errors += mpu_init(&params);
  errors += mpu_set_sensors(INV_XYZ_GYRO|INV_XYZ_ACCEL);
  errors += mpu_configure_fifo(INV_XYZ_GYRO|INV_XYZ_ACCEL);
  
  errors += mpu_get_gyro_sens(&gyro_sens);
  errors += mpu_get_accel_sens(&accel_sens);
  errors += mpu_set_sample_rate(4);
  Serial.print(errors); Serial.println(" errors.");

}

short gyro[3], accel[3];
unsigned char sensors;
void loop() {
  if(new_int){
    mpu_get_gyro_reg(gyro, NULL); mpu_get_accel_reg(accel, NULL);
    if(sensors & INV_XYZ_GYRO)
      Serial.print("Gyro: ");Serial.print(gyro[0]/gyro_sens);Serial.print(" ");Serial.print(gyro[1]/gyro_sens);Serial.print(" ");Serial.println(gyro[2]/gyro_sens);
    if(sensors & INV_XYZ_ACCEL)
      Serial.print("Acce: ");Serial.print(accel[0]/(float)accel_sens);Serial.print(" ");Serial.print(accel[1]/(float)accel_sens);Serial.print(" ");Serial.println(accel[2]/(float)accel_sens);
    Serial.println();
    new_int = false;
  }
}
