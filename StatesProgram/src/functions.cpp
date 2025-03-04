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
  double power = 0;
  L1.resetPosition();
  R1.resetPosition();
  while(error > 0.08*target) {
    double lefterror = (target - L1.position(deg));
    double righterror = (target - R1.position(deg));


    leftpower = 100*(lefterror)/abs(target);
    rightpower = 100*(righterror)/abs(target);
    error = (lefterror+righterror)/2.0;
    power = 100*(error)/abs(target);


    if(fabs((leftpower+rightpower)/2.0) < minimumSpeed) {
      if(target > 0){
        leftpower = minimumSpeed;
        rightpower = minimumSpeed;
      } else {
        leftpower = -minimumSpeed;
        rightpower = -minimumSpeed;
      }
    }


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


void correction(double target,double angle, double minimumSpeed) {
  double rightpower = 0;
  double leftpower = 0;
  double power = 0;
  int velocity_timer = 0;
  bool hasCompleted = true;
  Brain.Screen.clearScreen();
  double error = target - L1.position(deg);
  L1.resetPosition();
  R1.resetPosition();
  imu.resetRotation();
  double turn_error = angle - imu.rotation(deg);
  double turn_kP = (fabs(target)+fabs(angle))/1300;
  double turn_power = 0;
  while(error > 0.08*target) {


    turn_error = angle - inertialLimit();
    turn_power = turn_kP * turn_error;


    double lefterror = (target - L1.position(deg));
    double righterror = (target - R1.position(deg));
 
    error = (lefterror+righterror)/2.0;
    power = 100*error/fabs(target);


    if(fabs(power) < minimumSpeed) {
      if(target > 0){
        power = minimumSpeed;
      } else {
        power = -minimumSpeed;
      }
    }


    Brain.Screen.printAt(2,20,"%f",fabs(turn_error));
    Brain.Screen.printAt(4,40,"%f",fabs(L1.position(deg)));
    Brain.Screen.printAt(4,60,"%f",fabs(R1.position(deg)));
    runDrivetrain(forward, forward, 120*(power-turn_power), 120*(power+turn_power));
    wait(20, msec);
  }


  holdDrivetrain();
  Brain.Screen.clearScreen();
  wait(30,msec);
 
}
