#include "vex.h"
using namespace vex;

float inertialLimit() {
  float returnVal = 0;
  if(imu.rotation(deg) > 180) {
    returnVal = (imu.rotation(deg) - 360);
    //return limit;
  } else if(imu.rotation(deg) < -180) {
    returnVal = (imu.rotation(deg) + 360);
    //return limit;
  } else {
    returnVal = imu.rotation(deg);
  }
  return returnVal;
}

int sgn(float input) {
  if(input < 0) {
    return -1;    
  } else if(input > 0) {
    return 1;
  } else {
    return 0;
  }
}

