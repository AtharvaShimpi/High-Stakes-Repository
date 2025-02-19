#include "vex.h"

using namespace vex;

brain Brain;
inertial imu = inertial(PORT18);
digital_out clamp1 = digital_out(Brain.ThreeWirePort.A);
digital_out clamp2 = digital_out(Brain.ThreeWirePort.C);
digital_out doinker = digital_out(Brain.ThreeWirePort.H);
controller Controller1;
motor L1 = motor(PORT1,ratio6_1,true);
motor L2 = motor(PORT2,ratio6_1,true);
motor L3 = motor(PORT3,ratio6_1,true);
motor R1 = motor(PORT6,ratio6_1,false);
motor R2 = motor(PORT8,ratio6_1,false);
motor R3 = motor(PORT7,ratio6_1,false);
motor Intake = motor(PORT19,ratio6_1,true);
motor LadyBrown = motor(PORT10,ratio18_1,false);