#include "readMPUSensor.h"


u8 MPUSensorInit(void) {
  halI2CInit();
  while (halMPUInit() != 0);
  return 0;
}

typeMPUSensor ReadMPUSensor(void){
  return halReadMPUSensor();
}

typeMagSensor ReadMagSensor(void) {
  return halReadMagSensor();
}

static typeCalMagSensor calMagSensorPara;
typeCalMagSensor CallibrateMagSensor(u16 Xmax, u16 Xmin, u16 Ymax, u16 Ymin){
  float Xmid, Ymid;
  Xmid = (Xmax+Xmin)/2.0f;
  Ymid = (Ymax+Ymin)/2.0f;
  calMagSensorPara.x = Xmid;
  calMagSensorPara.y = Ymid;
  return calMagSensorPara;
}

