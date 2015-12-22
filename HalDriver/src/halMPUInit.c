#include "halMPUInit.h"

static signed char gyro_orientation[9] = {-1, 0, 0, 0,-1, 0, 0, 0, 1};
unsigned short inv_row_2_scale(const signed char *row) { 
   unsigned short b;
   if (row[0] > 0)         b = 0;
   else if (row[0] < 0)    b = 4;
   else if (row[1] > 0)    b = 1;
   else if (row[1] < 0)    b = 5;
   else if (row[2] > 0)    b = 2;
   else if (row[2] < 0)    b = 6;
   else                    b = 7;      // error
   return b;
}

unsigned short inv_orientation_matrix_to_scalar(const signed char *mtx)  {
  unsigned short scalar;
  scalar = inv_row_2_scale(mtx);
  scalar |= inv_row_2_scale(mtx + 3) << 3;
  scalar |= inv_row_2_scale(mtx + 6) << 6;
  return scalar;
}

void run_self_test(void) {
  int result;
  long gyro[3], accel[3];
  result = mpu_run_self_test(gyro, accel);
  if (result == 0x7) {
    float sens;
    unsigned short accel_sens;
    mpu_get_gyro_sens(&sens);
    gyro[0] = (long)(gyro[0] * sens);
    gyro[1] = (long)(gyro[1] * sens);
    gyro[2] = (long)(gyro[2] * sens);
    dmp_set_gyro_bias(gyro);
    mpu_get_accel_sens(&accel_sens);
    accel[0] *= accel_sens;
    accel[1] *= accel_sens;
    accel[2] *= accel_sens;
    dmp_set_accel_bias(accel);
    //PrintChar("setting bias succesfully ......\n");
  }
  else   {
    //PrintChar("bias has not been modified ......\n");
  }
}

////////////////////////////////////////////////////////////////////////////////
//初始化MPUsensor
////////////////////////////////////////////////////////////////////////////////
struct int_param_s int_param;
u8 halMPUInit(void){
  u8 state[8];
  state[0] = mpu_init(&int_param);
  state[1] = mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL | INV_XYZ_COMPASS);
  state[2] = mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL | INV_XYZ_COMPASS);
  state[3] = mpu_set_sample_rate(1000/MPU_SAMPLE_PERIOD); 
  state[4] = dmp_load_motion_driver_firmware();
  state[5] = dmp_set_orientation(inv_orientation_matrix_to_scalar(gyro_orientation));
  state[6] = dmp_enable_feature(DMP_FEATURE_6X_LP_QUAT     | DMP_FEATURE_TAP |
	                        DMP_FEATURE_ANDROID_ORIENT | DMP_FEATURE_SEND_RAW_ACCEL | DMP_FEATURE_SEND_CAL_GYRO |
	                        DMP_FEATURE_GYRO_CAL);
  
  state[7] = dmp_set_fifo_rate(1000/MPU_SAMPLE_PERIOD);
    
  run_self_test();
  mpu_set_dmp_state(1);
  
  return (state[0]|state[1]|state[2]|state[3]|state[4]|state[5]|state[6]|state[7]);
}

////////////////////////////////////////////////////////////////////////////////
//读取accel, gyro sensor
////////////////////////////////////////////////////////////////////////////////
static typeMPUSensor mpuSensor;
typeMPUSensor halReadMPUSensor(void) {
  short sensors;
  u8 more;
  short acc[3],gyro[3];
  long quat[4];
  unsigned long sensor_timestamp;
  float q0, q1, q2, q3;
  
  dmp_read_fifo(gyro, acc, quat, &sensor_timestamp, &sensors, &more);
  
  if (sensors & INV_WXYZ_QUAT) {
    q0=quat[0] / q30;
    q1=quat[1] / q30;
    q2=quat[2] / q30;
    q3=quat[3] / q30;
    mpuSensor.pitch  = asin(2 * q1 * q3 - 2 * q0* q2)* 57.3;                                       // pitch
    mpuSensor.roll   = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2* q2 + 1)* 57.3;      // roll
    mpuSensor.yaw    = atan2(2*(q1*q2 + q0*q3),q0*q0+q1*q1-q2*q2-q3*q3) * 57.3;                    //Yaw
    mpuSensor.accelX =  acc[0]  / 16384.0;
    mpuSensor.accelY =  acc[1]  / 16384.0;
    mpuSensor.accelZ =  acc[2]  / 16384.0;
    mpuSensor.gyroX  =  gyro[0] / 16.4;
    mpuSensor.gyroY  =  gyro[1] / 16.4;
    mpuSensor.gyroZ  =  gyro[2] / 16.4;
    mpuSensor.timeStamp =  sensor_timestamp;
  }
  return mpuSensor;
}

////////////////////////////////////////////////////////////////////////////////
//读取mag sensor
////////////////////////////////////////////////////////////////////////////////
static typeMagSensor magSensor;
typeMagSensor halReadMagSensor(void) {
  static signed short compass[3];
  unsigned long sensor_timestamp;
  
  if (mpu_get_compass_reg(compass, &sensor_timestamp) != 0){
    return magSensor;
  }
  magSensor.magX = compass[0];
  magSensor.magY = compass[1];
  magSensor.magZ = compass[2];
  magSensor.timeStamp = sensor_timestamp;
  return magSensor;
}

