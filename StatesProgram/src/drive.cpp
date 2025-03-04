#include "vex.h"
using namespace vex;

void runDrivetrain(vex::directionType rightDirection, vex::directionType leftDirection, double rightPower, double leftPower) {
    L1.spin(leftDirection,leftPower,vex::voltageUnits::mV);
    L2.spin(leftDirection,leftPower,vex::voltageUnits::mV);
    L3.spin(leftDirection,leftPower,vex::voltageUnits::mV);
    R1.spin(rightDirection,rightPower,vex::voltageUnits::mV);
    R2.spin(rightDirection,rightPower,vex::voltageUnits::mV);
    R3.spin(rightDirection,rightPower,vex::voltageUnits::mV);
}

void holdDrivetrain(){
  L1.stop(hold);
  L2.stop(hold);
  L3.stop(hold);
  R1.stop(hold);
  R2.stop(hold);
  R3.stop(hold);
}