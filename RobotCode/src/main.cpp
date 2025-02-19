/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       athar                                                     */
/*    Created:      9/16/2024, 7:02:59 PM                                     */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "vex.h"

using namespace vex;

// A global instance of competition
competition Competition;
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
//enum 
enum wallStakeStates {
  Idle,
  Loading,
  Scoring,
  Reset,
};

struct wallStake {
  wallStakeStates states;
  bool isCompleted;
};
wallStake wallstake;
//rotation Rotation = rotation(PORT1,false);
int autonum = 5;


// define your global instances of motors and other devices here

/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  You may want to perform some actions before the competition starts.      */
/*  Do them in the following function.  You must return from this function   */
/*  or the autonomous and usercontrol tasks will not be started.  This       */
/*  function is only called once after the V5 has been powered on and        */
/*  not every time that the robot is disabled.                               */
/*---------------------------------------------------------------------------*/

void pre_auton(void) {
  imu.calibrate();  
  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ...
}

void runDrivetrain(vex::directionType rightDirection, vex::directionType leftDirection, float rightPower, float leftPower) {
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

bool isMotorStalling(float velocity, float stallVelocityThreshold, int stallTimeThresholdMs) {
    /**
     * Checks if a motor is stalling based on velocity alone.
     *
     * @param velocity: The velocity of the motor (e.g., in rad/s or m/s).
     * @param stallVelocityThreshold: The velocity below which the motor is considered stalled.
     * @param stallTimeThresholdMs: The minimum time (in milliseconds) the motor must be below the velocity threshold to be considered stalled.
     * @return: True if the motor is stalling, False otherwise.
     */
    
    static auto stallStartTime = std::chrono::steady_clock::now();
    static bool belowThreshold = false;

    if (velocity <= stallVelocityThreshold) {
        if (!belowThreshold) {
            // First time going below threshold, record time
            stallStartTime = std::chrono::steady_clock::now();
            belowThreshold = true;
        } else {
            // Check if the motor has been below the threshold long enough
            auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now() - stallStartTime
            ).count();

            if (elapsedTime >= stallTimeThresholdMs) {
                return true; // Motor is stalling
            }
        }
    } else {
        // Motor is moving normally, reset the timer
        belowThreshold = false;
    }

    return false;
}

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
 // runDrivetrain(forward,forward,0,0); verify and delete
  Brain.Screen.clearScreen();
  wait(30,msec);
}
void drive_PNew(double target,double minimumSpeed,int voltageProportion)
{
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
//gives the sign of the inputted variable (negative or positive)
int sgn(float input) {
  if(input < 0) {
    return -1;    
  } else if(input > 0) {
    return 1;
  } else {
    return 0;
  }
}

/*
void angle (double angle) {
  imu.setRotation((imu.rotation(deg)-angle),deg);
}
*/

//turns the robot based on its absolute degree
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

//makes values over 180 negative
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

//turn Proportion loop
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
int intakeTaskUserControl() {
  while(true) {
    if(isMotorStalling(Intake.velocity(pct),0,100) && Intake.current(pct) > 0) {
      Intake.spin(reverse,12000,vex::voltageUnits::mV);
      wait(100,msec);
    }
  }
  return 0;
}
struct RIntake 
{
  bool isRunningIntake;
};
RIntake intake;
int intakeTask() {
  while(true) {
    if(isMotorStalling(Intake.velocity(pct),0,100) && Intake.current(pct) > 0) {
      Intake.spin(reverse,12000,vex::voltageUnits::mV);
      wait(500,msec);
      Intake.spin(forward,0,vex::voltageUnits::mV);
    } else if (intake.isRunningIntake == true && !isMotorStalling(Intake.velocity(pct),0,100)) {
      Intake.spin(forward,12000,vex::voltageUnits::mV);
    } else {
      Intake.spin(forward,0,vex::voltageUnits::mV);
    }
  }
  return 0;
}
void soloAWPStates () {
  vex::task runIntake1(intakeTask);
  intake.isRunningIntake = false;
  L1.resetPosition(); 
  R1.resetPosition();
  LadyBrown.resetPosition();
  drive_P(200,40,120);
  wait(200,msec);
  moveWallStake(350,wallstake,12000);
  clamp1.set(true);
  clamp2.set(true);
  correction(-300,17,0);
  drive_P(-1800,100,120);
  clamp1.set(false);
  clamp2.set(false);
  wait(500,msec);
  turn_P(160,10);
  drive_P(600,20,100);
  intake.isRunningIntake = true;
  correction(100,-50,15);
  drive_P(600,20,100);
  wait(500,msec);
  correction(-100,50,15);
  drive_P(-700,10,100);
  turn_P(-45,10);
  drive_P(400,10,100);
  wait(500,msec);
  drive_P(-400,10,100);
  turn_P(-90,6);
  drive_P(600,10,100);
  /*
  turn_P(-40,6);
  clamp1.set(true);
  clamp2.set(true);
  Intake.spin(forward,6000,vex::voltageUnits::mV);
  drive_P(2000,10,100);
  Intake.spin(forward,0,vex::voltageUnits::mV);
  Intake.spin(reverse,12000,vex::voltageUnits::mV);
  wait(200,msec);
  Intake.spin(forward,0,vex::voltageUnits::mV);
  turn_P(75,5);
  drive_P(-1200,10,100);
  clamp1.set(false);
  clamp2.set(false);
  wait(200,msec);
  turn_P(180,10);
  drive_P(300,10,100);
  */
}

void red1() {
  clamp1.set(true);
  clamp2.set(true);
  drive_P(-1600,10,90);
  clamp1.set(false);
  clamp2.set(false);
  wait(300,msec);
  turn_P(-45,10);
  Intake.spin(forward,12000,vex::voltageUnits::mV);
  wait(500,msec);
  drive_P(1200,10,120);
  wait(500,msec);
  drive_P(-1200,10,120);
  clamp1.set(true);
  clamp2.set(true);
  drive_P(1000,20,120);
  turn_P(90,10);
}

void rush() {
  drive_P(200,10,100);
  turn_P(45,2);
  //set wallstake
  clamp1.set(true);
  clamp2.set(true);
  drive_P(-3000,10,100);  
  clamp1.set(false);
  clamp2.set(false);
  turn_P(180,10);
  drive_P(500,10,100);
  turn_P(90,10);
  drive_P(100,10,100);
  turn_P(-45,10);
  drive_P(-200,10,100);

}
void blue1(){

  clamp1.set(true);
  clamp2.set(true);
  drive_P(-1600,10,90);
  clamp1.set(false);
  clamp2.set(false);
  wait(300,msec);
  turn_P(45,10);
  Intake.spin(forward,12000,vex::voltageUnits::mV);
  wait(500,msec);
  drive_P(1200,10,120);
  wait(500,msec);
  drive_P(-1200,10,120);
  clamp1.set(true);
  clamp2.set(true);
  drive_P(1000,20,120);
  turn_P(-90,10);

  
}

void red4Point(){
  L1.resetPosition(); 
  R1.resetPosition();
  LadyBrown.resetPosition();
  drive_P(300,40,120);
  turn_P(-6,10);
  moveWallStake(450,wallstake,10000);
  wait(200,msec);
  turn_P(10,10);
  clamp1.set(true);
  clamp2.set(true);
  correction(-300,17,0);
  Intake.spin(forward,12000,vex::voltageUnits::mV);
  moveWallStake(100,wallstake,10000);
  drive_P(-1800,60,120);
  clamp1.set(false);
  clamp2.set(false);
  wait(500,msec);
  turn_P(160,10);
  drive_P(700,15,100);
  correction(100,-50,15);
  drive_P(800,15,100); 
  drive_P(-200,15,100);
  wait(500,msec);
  correction(-90,50,15);
  drive_P(-700,10,100);
  turn_P(-45,10);
  drive_P(400,10,100);
  wait(500,msec);
  drive_P(-400,10,100);
  turn_P(170,20);
  moveWallStake(-300,wallstake,10000);
  drive_P(1000,10,120);
}

void blue2() {
  L1.resetPosition(); 
  R1.resetPosition();
  LadyBrown.resetPosition();
  drive_P(300,40,120);
  turn_P(6,10);
  moveWallStake(450,wallstake,10000);
  wait(200,msec);
  turn_P(-10,10);
  clamp1.set(true);
  clamp2.set(true);
  correction(-300,-17,0);
  Intake.spin(forward,12000,vex::voltageUnits::mV);
  moveWallStake(100,wallstake,10000);
  drive_P(-1800,60,120);
  clamp1.set(false);
  clamp2.set(false);
  wait(500,msec);
  turn_P(-160,10);
  drive_P(700,15,100);
  correction(100,50,15);
  drive_P(800,15,100); 
  drive_P(-200,15,100);
  wait(500,msec);
  correction(-90,-50,15);
  drive_P(-700,10,100);
  turn_P(45,10);
  drive_P(400,10,100);
  wait(500,msec);
  drive_P(-400,10,100);
  turn_P(-170,20);
  moveWallStake(-300,wallstake,10000);
  drive_P(1000,10,120);
  /*
  drive_P(-1000,7,100);
  drive_P(200,10,100);
  turn_P(-75,2);
  drive_P(-400,10,100);
  wait(250,msec);
 Intake.spin(forward,100,pct);
  wait(700,msec);
 Intake.spin(reverse,100,pct);
  drive_P(600,1,100);
  clamp1.set(true);
  clamp2.set(true);
  turn_P(-145,10);
  Intake.spin(forward,100,pct);
  drive_P(-1750,14,100);
  clamp1.set(false);
  clamp2.set(false);
  wait(200,msec);
  turn_P(-135,10);
  Intake.spin(forward,12000,vex::voltageUnits::mV);
  drive_P(900,14,100);
   wait(200,msec);
  turn_P(-80,10);
  drive_P(500,14,100);
   wait(200,msec);
  drive_P(-650,14,100);
  turn_P(15,10);
  drive_P(900,14,100);
   wait(200,msec);
  drive_P(-900,15,100);
  turn_P(-120,20);
  drive_P(2000,15,100);
  */
}

void skills () {
  Intake.spin(forward,12000,vex::voltageUnits::mV);
  wait(500,msec);
  drive_P(600,20,120);
  turn_P(-80,10);
  clamp1.set(true);
  clamp2.set(true);
  drive_P(-1200,10,120);
  clamp1.set(false);
  clamp2.set(false);
  wait(500,msec);
  turn_P(170,20);
  drive_P(1000,10,120);
  turn_P(-140,10);
  drive_P(-1000,10,100);
  clamp1.set(true);
  clamp2.set(true);
  drive_P(750,10,100);
  turn_P(130,10);
  drive_P(-2000,10,120);
  clamp1.set(false);
  clamp2.set(false);
  /*
  Intake.spin(forward,12000,vex::voltageUnits::mV);
  wait(500,msec);
  clamp1.set(true);
  clamp2.set(true);
  wait(200,msec);
  drive_P(500,10,120);
  turn_P(-90,10);
  drive_P(-1200,10,120);
  clamp1.set(false);
  clamp2.set(false);
  wait(200,msec);
  turn_P(90,10);
  drive_P(800,10,120);
  turn_P(10,10);
  LadyBrown.resetPosition();
  drive_P(2500,60,120);
  intake.isRunningIntake = false;
  wait(500,msec);
  moveWallStake(100,wallstake,12000);
  drive_P(-1000,10,120);
  turn_P(50,10);
  LadyBrown.resetPosition();
  moveWallStake(40,wallstake,10000);
  intake.isRunningIntake = true;
  drive_P(1000,10,120);
  moveWallStake(200,wallstake,10000);
  */
}

void runAuto(int i){
  switch(i)
  {
    case 1:
      soloAWPStates();
      break;
    case 2: 
      red1();
      break;
    case 3:
      blue1();
      break;
    case 4:
      red4Point();
      break;
    case 5:
      blue2();
      break;
    case 6:
      skills();
      break;
  }
}

void autonomous(void) {
  runAuto(autonum);
}

void usercontrol(void) {
  // User control code here, inside the loop
   //defines Toggle variables
  bool isToggled = false;
  bool isToggled1 = false;
  bool isToggled2 = false;
  bool prevValue;
  bool prevValue1;
  bool prevValue2;
//sets Motors to brake to enhance defensive capabilites
 // vex::task IntakeStall(intakeTaskUserControl);
  LadyBrown.resetPosition();
  vex::task wallTask(wallStakeTask);
  int stall = 0;
  while (1) {
    int rightPower = Controller1.Axis2.position()*120;
    int leftPower = Controller1.Axis3.position()*120;
    L1.spin(forward, leftPower,voltageUnits::mV);
    L2.spin(forward, leftPower,voltageUnits::mV);
    L3.spin(forward, leftPower,voltageUnits::mV);
    R1.spin(forward, rightPower,voltageUnits::mV);
    R2.spin(forward, rightPower,voltageUnits::mV);
    R3.spin(forward, rightPower,voltageUnits::mV); 
    //replace with function
    if(Controller1.ButtonRight.pressing()) {
      LadyBrown.spin(forward, 12000,vex::voltageUnits::mV);
    } else if(Controller1.ButtonDown.pressing()) {
      LadyBrown.spin(reverse, 12000,vex::voltageUnits::mV);
    } else {
      LadyBrown.stop(hold);
    }
    if(Controller1.ButtonR1.pressing()){
      Intake.spin(forward, 12000,vex::voltageUnits::mV);
    } else if(Controller1.ButtonR2.pressing()) {
      Intake.spin(reverse, 12000,vex::voltageUnits::mV);
    } else {
      Intake.spin(reverse, 0,vex::voltageUnits::mV);
    }
    if (Controller1.ButtonL1.pressing() && !prevValue2) {
      isToggled2 = !isToggled2;
    }
    prevValue2 = Controller1.ButtonL1.pressing();
    //if button is pressed once, horizontal wings open
    if (isToggled2) {
      clamp1.set(false);
      clamp2.set(false);
    //if button is pressed again, horizontal wings close
    } else {
      clamp1.set(true);
      clamp2.set(true);
    }
    if (Controller1.ButtonL2.pressing() && !prevValue) {
      isToggled = !isToggled;
    }
    prevValue = Controller1.ButtonL2.pressing();
    //if button is pressed once, horizontal wings open
    if (isToggled) {
      doinker.set(true);
    //if button is pressed again, horizontal wings close
    } else {
      doinker.set(false);
    }
    wait(20, msec); // Sleep the task for a short amount of time to
                    // prevent wasted resources.
  }
}

//
// Main will set up the competition functions and callbacks.
//
int main() {
  // Set up callbacks for autonomous and driver control periods.
 
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  // Run the pre-autonomous function.
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true) {
    Controller1.Screen.newLine();
    Controller1.Screen.print(autonum);
    if(Brain.Screen.pressing()) {
     wait(500,msec);
     autonum++;
    }
    if(autonum > 6) {
      autonum = 1;
    }
    wait(100, msec);
  }
}
