#include "vex.h"
using namespace vex;

void moveWallStake (double target, wallStake wallstake, double power) {
  if(LadyBrown.position(deg) < (target-5) && LadyBrown.position(deg) > (target+5)) {
    return;
  }
  if(target > LadyBrown.position(deg)) {
    while(fabs(LadyBrown.position(deg)) < fabs(target)) { 
      LadyBrown.spin(fwd,power,voltageUnits::mV);
    }
     LadyBrown.spin(fwd,0,voltageUnits::mV);
  } else if (target < LadyBrown.position(deg)) {
    while(fabs(LadyBrown.position(deg)) < fabs(target)) { 
        LadyBrown.spin(reverse,power,voltageUnits::mV);
     }
     LadyBrown.spin(fwd,0,voltageUnits::mV);
  }
  wallstake.isCompleted = true;
}

int wallStakeTask() {
  int value = 0;
  while(true) {
      if(Controller1.ButtonY.pressing()) {
        value = 1;
      } else {
        value = 0;
      }
    switch(wallstake.states) {
      case Idle:
        if(value == 1) {
          Brain.Screen.printAt(2,20,"Loading");
          wallstake.isCompleted = false;
          wallstake.states = Loading;
          wait(250,msec);
        }
      break;
      case Loading:
        if(!wallstake.isCompleted) {
          moveWallStake(100,wallstake,10000);
          wallstake.isCompleted = true;
        } else {
          wallstake.states = Idle;
        }
        /*
        if(value == 1 && wallstake.isCompleted) {
          Brain.Screen.printAt(2,20,"Scoring");
          LadyBrown.resetPosition();
          wallstake.isCompleted = false;
          wait(250,msec);
        }
        */
      break;
      case Scoring:
        while(!wallstake.isCompleted) {
          moveWallStake(750,wallstake,10000);
        }
        if(value == 1 && wallstake.isCompleted) {
          Brain.Screen.printAt(2,20,"Reset");
          LadyBrown.resetPosition();
          wallstake.isCompleted = false;
          wallstake.states = Reset; 
          wait(250,msec);
        }
      break;
      case Reset:
        while(!wallstake.isCompleted) {
        moveWallStake(-1000,wallstake,10000);
        }
        if(value == 1 && wallstake.isCompleted) {
          LadyBrown.resetPosition();
          wallstake.isCompleted = false;
          Brain.Screen.printAt(2,20,"Idle");
          wallstake.states = Idle;
        }
      break;
    }
  }
  return 0;
}