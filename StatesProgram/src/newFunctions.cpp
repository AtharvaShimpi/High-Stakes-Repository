#include "vex.h"
using namespace vex;

void drive_PNew(double target,double minimumSpeed,int voltageProportion) {
  double rightpower = 0;
  double leftpower = 0;
  bool hasCompleted = true;
  Brain.Screen.clearScreen();
  double error = target - L1.position(deg);
  double kp = 100.0/(target);
  L1.resetPosition();
  R1.resetPosition();
  while(hasCompleted) {
    double lefterror = (target - L1.position(deg));
    double righterror = (target - R1.position(deg));

    if(fabs(error) == 0) { 
        hasCompleted = false;
    } else if ((L1.velocity(pct) == 0 && fabs(error) < 0.2*abs(target)) || (R1.velocity(pct) == 0 && fabs(error) < 0.2*abs(target))) {
        hasCompleted = false;
    }

    leftpower = fabs(lefterror)*kp;
    rightpower = fabs(righterror)*kp;
    error = (lefterror+righterror)/2.0;

    if(fabs((leftpower+rightpower)/2.0) < minimumSpeed) { 
      if(target > 0){
        leftpower = minimumSpeed;
        rightpower = minimumSpeed;
      } else {
        leftpower = -minimumSpeed;
        rightpower = -minimumSpeed;
      }
    }

    float power = 100*(error)/abs(target);
    Brain.Screen.printAt(2,20,"%f",fabs(error));
    Brain.Screen.printAt(4,40,"%f",fabs(L1.position(deg)));
    Brain.Screen.printAt(4,60,"%f",fabs(R1.position(deg)));
    runDrivetrain( forward, forward, voltageProportion*power, voltageProportion*power);
    wait(20, msec);
  }

  holdDrivetrain();
 // runDrivetrain(forward,forward,0,0); verify and delete
  Brain.Screen.clearScreen();
  wait(30,msec);
}

void universalTurnFunction(double target, float minimumSpeed) {
  double power = 0; 
  int velocity_timer = 0;
  bool hasCompleted = true;
  double realTarget = target + imu.rotation(deg);
  double error = realTarget - imu.rotation(deg);
  double kP = fabs(error); 

  while(fabs(error) > 2) { 
    power = (100/kP) * error; 
    error = realTarget - imu.rotation(deg);

    if(fabs(power) < minimumSpeed) { 
      if(power > 0) {
        power = minimumSpeed;
      } else if(power < 0) {
        power = -minimumSpeed;
      }
    }
    runDrivetrain(reverse,forward,110*power,110*power);
    wait(10, msec);
  }

  holdDrivetrain();
  wait(50,msec);
}