#include "vex.h"
using namespace vex;

void drive_P(int target,double minimumSpeed,int voltageProportion)
{
  double rightpower = 0;
  double leftpower = 0;
  int velocity_timer = 0;
  bool hasCompleted = true;
  Brain.Screen.clearScreen();
  double error = target - L1.position(deg);
  L1.resetPosition(); 
  R1.resetPosition();
  while(hasCompleted) {
    double lefterror = (target - L1.position(deg));
    double righterror = (target - R1.position(deg));

    if(target > 0 && error <= 0) { 
        hasCompleted = false;
    } else if(target < 0 && error >= 0) {
        hasCompleted = false;
    } else if ((L1.velocity(pct) == 0 && fabs(error) < 0.4*abs(target)) || (R1.velocity(pct) == 0 && fabs(error) < 0.4*abs(target))) {
        hasCompleted = false;
    }

    leftpower = 100*(lefterror)/abs(target);
    rightpower = 100*(righterror)/abs(target);
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
  Brain.Screen.clearScreen();
  wait(30,msec);
}

void turn_P(int target,int minimumSpeed) {
  double power = 0;
  int velocity_timer = 0;
  bool hasCompleted = true;
  imu.resetRotation();
  double error = target - imu.rotation(deg);
  double kP = fabs(error); 

  while(fabs(error) > 5) {
    error = target - inertialLimit();
    power =(100/kP)  * error; 
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

void correction(int target,int angle, int minimumSpeed) {

  double power = 0;
  int velocity_timer = 0;
  L1.resetPosition();
  R1.resetPosition();
  imu.resetRotation();
  double lefterror = 0;
  double righterror = 0;
  double error = target - R1.position(deg);
  bool hasCompleted = true;
  double turn_power = 0;
  double turn_error = angle - imu.rotation();
  double kP = fabs(turn_error); 

  while(fabs(error) > 2 && fabs(turn_error) > 1) {
    turn_error = angle - inertialLimit();
    turn_power =(100/kP)  * turn_error; 

    if(fabs(turn_power) < minimumSpeed) { 
      if(turn_power > 0) {
        turn_power = minimumSpeed;
      } else if(turn_power < 0) {
        turn_power = -minimumSpeed;
      }
    }

    if (turn_error > 0 && error < 0){
      error = (target - L1.position(deg));
    } else if (turn_error < 0 && error < 0){
     error = target - R1.position(deg);
    } else if (turn_error > 0 && error > 0){
     error = target - R1.position(deg);
    } else if (turn_error < 0 && error > 0){
      error = target - L1.position(deg);
    } else {
      error = target - L1.position(deg);
    }

    power = ((100*error)/fabs(target));

    if(fabs(power) < minimumSpeed){ 
      if(power > 0){
        power = minimumSpeed;
      } else {
        power = -minimumSpeed;
      }
    }

    float power1 = power + (turn_power);
    float power2 = power - (turn_power);
    
    if(power1 > 100){
      power1 = 90;
    }
    if(power2 > 100){
      power2 = 90;
    }

    runDrivetrain(forward,forward,90*power2,90*power1);
    wait(10,msec);
  }
  holdDrivetrain();
  wait(100,msec);
}