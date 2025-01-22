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
inertial imu = inertial(PORT4);
distance dsensor = distance(PORT20);
digital_out clamp1 = digital_out(Brain.ThreeWirePort.A);
digital_out clamp2 = digital_out(Brain.ThreeWirePort.C);
digital_out doinker = digital_out(Brain.ThreeWirePort.E);
controller Controller1;
motor L1 = motor(PORT1,ratio6_1,true);
motor L2 = motor(PORT2,ratio6_1,true);
motor L3 = motor(PORT3,ratio6_1,true);
motor R1 = motor(PORT6,ratio6_1,false);
motor R2 = motor(PORT9,ratio6_1,false);
motor R3 = motor(PORT7,ratio6_1,false);
motor Intake = motor(PORT6,ratio6_1,true);
//motor LadyBrown = motor(PORT20,ratio6_1,true);
//rotation Rotation = rotation(PORT1,false);
int autonum = 1;



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
void drive_P(int target,int minimumSpeed)
{
  float rightpower = 0;
  float leftpower = 0;
  int velocity_timer = 0;
  bool hasCompleted = true;
  L1.resetPosition(); 
  R1.resetPosition();
  while(hasCompleted)
  {
    float lefterror = (target - L1.position(deg));
    float righterror = (target - R1.position(deg));
        
    leftpower = ((100*lefterror)/abs(target));
    rightpower = ((100*righterror)/abs(target));
    if(abs(leftpower) < minimumSpeed || abs(rightpower) < minimumSpeed) 
    { 
      if(leftpower > 0 && rightpower > 0)
      {
        leftpower = minimumSpeed;
        rightpower = minimumSpeed;
      }else
      {
        leftpower = -minimumSpeed;
        rightpower = -minimumSpeed;
      }
    }
    L1.spin(forward,120*leftpower,vex::voltageUnits::mV);
    L2.spin(forward,120*leftpower,vex::voltageUnits::mV);
    L3.spin(forward,120*leftpower,vex::voltageUnits::mV);
    R1.spin(fwd,120*rightpower,vex::voltageUnits::mV);
    R2.spin(fwd,120*rightpower,vex::voltageUnits::mV);
    R3.spin(fwd,120*rightpower,vex::voltageUnits::mV);
    if(abs(lefterror) < 40 && abs(righterror) < 40) {
      hasCompleted = false;
      vexDisplayErase();
    } 
    wait(10, msec);
  }
  L1.stop(hold);
  L2.stop(hold);
  L3.stop(hold);
  R1.stop(hold);
  R2.stop(hold);
  R3.stop(hold);
  wait(50,msec);
}
int sgn(float input)
{
  if(input < 0)
  {
    return -1;    
  }
  else if(input > 0)
  {
    return 1;
  }
  else
  {
    return 0;
  }
}
float inertialLimit()
{
  if(imu.rotation(deg) > 180)
  {
    float limit  = (imu.rotation(deg) - 360);
    return limit;
  }
  else if(imu.rotation(deg) < -180)
  {
    float limit = (imu.rotation(deg) + 360);
    return limit;
  }
  else
  {
    return imu.rotation(deg);
  }
}
void turn_P(int target,int minimumSpeed)
{

  float power = 0;
  int velocity_timer = 0;
  bool hasCompleted = true;
  imu.resetRotation();
  float error = target - imu.rotation(deg);
  float kP = abs(error); 
  while(hasCompleted)
  {
    error = target - inertialLimit();
    power =(100/kP)  * error; 
    if(abs(power) < minimumSpeed) 
    { 
      if(power > 0)
      {
        power = minimumSpeed;
      }
      else if(power < 0)
      {
        power = -minimumSpeed;
      }
    }
    L1.spin(forward,90*power,vex::voltageUnits::mV);
    R1.spin(reverse,90*power,vex::voltageUnits::mV);
    L2.spin(forward,90*power,vex::voltageUnits::mV);
    R2.spin(reverse,90*power,vex::voltageUnits::mV);
    L3.spin(forward,90*power,vex::voltageUnits::mV);
    R3.spin(reverse,90*power,vex::voltageUnits::mV);
    if(abs(error) < 5) {
      hasCompleted = false;
      vexDisplayErase();
    }
    wait(10, msec);
  }
  L1.stop(hold);
  L2.stop(hold);
  L3.stop(hold);
  R1.stop(hold);
  R2.stop(hold);
  R3.stop(hold);
  wait(50,msec);
}
/*
enum wallStakeStates
{
  Idle,
  Loading1,
  Loading2,
  Scoring,
  Reset,
};
struct wallStake
{
  wallStakeStates states;
  bool isCompleted;
};
wallStake wallstake;
*/

void correction(int target,int angle, int minimumSpeed)
{
  float power = 0;
  float power = 0;
  float turn_power = 0;
  int velocity_timer = 0;
  float error = target - L1.position(deg);
  bool hasCompleted = true;
  imu.resetRotation();
  float turn_error = angle - inertialLimit();
  float kP = abs(turn_error); 
  while(hasCompleted)
  {
    turn_error = angle - inertialLimit();
    turn_power =(100/kP)  * turn_error; 
    if(abs(turn_power) < minimumSpeed) 
    { 
      if(turn_power > 0)
      {
        turn_power = minimumSpeed;
      }
      else if(turn_power < 0)
      {
        turn_power = -minimumSpeed;
      }

    }
    float error = (target - L1.position(deg));
        
    power = ((100*error)/abs(target));
    if(abs(power) < minimumSpeed || abs(power) < minimumSpeed) 
    { 
      if(power > 0 && power > 0)
      {
        power = minimumSpeed;
        power = minimumSpeed;
      }else
      {
        power = -minimumSpeed;
        power = -minimumSpeed;
      }
    }
    L1.spin(forward,90*(power+turn_power),vex::voltageUnits::mV);
    R1.spin(reverse,90*(power-turn_power),vex::voltageUnits::mV);
    L2.spin(forward,90*(power+turn_power),vex::voltageUnits::mV);
    R2.spin(reverse,90*(power-turn_power),vex::voltageUnits::mV);
    L3.spin(forward,90*(power+turn_power),vex::voltageUnits::mV);
    R3.spin(reverse,90*(power-turn_power),vex::voltageUnits::mV);

  L1.stop(hold);
  L2.stop(hold);
  L3.stop(hold);
  R1.stop(hold);
  R2.stop(hold);
  R3.stop(hold);
  wait(50,msec);
  }
}
/*
void moveWallStake (float target, wallStake wallstake)
{
  int sgn;
  if(target > LadyBrown.position(deg))
  {
    int sgn = 1;
  }
  else
  {
    int sgn = -1;
  }
  wallstake.isCompleted = false;
  if(abs(LadyBrown.position(deg)) > target)
  {
    wallstake.isCompleted = true;
  }
  LadyBrown.spin(fwd,9600*sgn,voltageUnits::mV);
}
int buttonPressed(){
  if(Controller1.ButtonDown.pressing())
  {
    return 1;
  }
  else
  {
    return 0;
  }
}
int wallStakeTask ()
{
  while(true)
  {
    switch(wallstake.states)
    {
      case Idle:

      break;
      case Loading1:
        moveWallStake(200,wallstake);
        if(buttonPressed)
        {
          wallstake.states = Loading2;
          wait(250,msec);
        }
      break;
      case Loading2:
        moveWallStake(300,wallstake);
        if(buttonPressed)
        {
          wallstake.states = Scoring;
          wait(250,msec);
        }
      break;
      case Scoring:
        moveWallStake(5000,wallstake);
        if(buttonPressed)
        {
          wallstake.states = Reset;
          wait(250,msec);
        }
      break;
      case Reset:
        moveWallStake(-7500,wallstake);
        if(wallstake.isCompleted)
        {
          wallstake.states = Idle;
        }
      break;
    }
      wallstake.states;

  }
  return 0;
}
*/
void Red1 (){
  /*
clamp1.set(true);
 clamp2.set(true);
 drive_P(-1600,10);
 clamp1.set(false);
 clamp2.set(false);
 turn_P(-70,2);
 Intake.spin(forward,12000,vex::voltageUnits::mV);
 wait(500,msec);
  drive_P(1200,10);
  wait(500,msec);
    turn_P(186,2);
    drive_P(1500,10);
    */
   correction(100,0,10);
}
void Blue1(){
  clamp1.set(true);
  clamp2.set(true);
  drive_P(-1600,10);
  clamp1.set(false);
  clamp2.set(false);
  turn_P(70,2);
  Intake.spin(forward,12000,vex::voltageUnits::mV);
  wait(500,msec);
  drive_P(1200,10);
  wait(500,msec);
  turn_P(180,2);
}
void Red2(){
  /*
  clamp1.set(true);
  clamp2.set(true);
  drive_P(-1700,20);
  clamp1.set(false);
  clamp2.set(false);
  wait(250,msec);
  drive_P(200,10);
  turn_P(55,10);
  Intake.spin(forward,12000,vex::voltageUnits::mV);
  drive_P(700,10);
  turn_P(80,10);
  drive_P(500,10);
  drive_P(-650,10);
  turn_P(-15,10);
  drive_P(900,10);
  drive_P(-900,15);
  turn_P(120,20);
  drive_P(2000,10);
  */
  drive_P(-1000,7);
  drive_P(200,1);
  turn_P(90,1);
  drive_P(-400,1);
  wait(250,msec);
 Intake.spin(forward,100,pct);
  wait(700,msec);
 Intake.spin(reverse,100,pct);
  drive_P(600,10);
  clamp1.set(true);
  clamp2.set(true);
  turn_P(135,10);
  Intake.spin(forward,100,pct);
  drive_P(-1900,14);
  clamp1.set(false);
  clamp2.set(false);
  wait(200,msec);
  turn_P(135,10);
  Intake.spin(forward,12000,vex::voltageUnits::mV);
  drive_P(700,14);
   wait(200,msec);
  turn_P(80,10);
  drive_P(500,14);
   wait(200,msec);
  drive_P(-650,14);
  turn_P(-15,10);
  drive_P(900,14);
   wait(200,msec);
  drive_P(-900,15);
  turn_P(120,20);
  drive_P(2000,15);

}
void Blue2(){
  drive_P(-1000,7);
  drive_P(200,1);
  turn_P(-90,1);
  drive_P(-400,1);
  wait(250,msec);
 Intake.spin(forward,100,pct);
  wait(700,msec);
 Intake.spin(reverse,100,pct);
  drive_P(600,10);
  clamp1.set(true);
  clamp2.set(true);
  turn_P(-135,10);
  Intake.spin(forward,100,pct);
  drive_P(-1900,14);
  clamp1.set(false);
  clamp2.set(false);
  wait(200,msec);
  turn_P(-135,10);
  Intake.spin(forward,12000,vex::voltageUnits::mV);
  drive_P(700,14);
   wait(200,msec);
  turn_P(-80,10);
  drive_P(500,14);
   wait(200,msec);
  drive_P(-650,14);
  turn_P(15,10);
  drive_P(900,14);
   wait(200,msec);
  drive_P(-900,15);
  turn_P(-120,20);
  drive_P(2000,15);
  /*
  clamp1.set(true);
  clamp2.set(true);
  drive_P(-1700,8);
  clamp1.set(false);
  clamp2.set(false);
  wait(250,msec);
  drive_P(200,8);
  turn_P(-55,10);
  Intake.spin(forward,12000,vex::voltageUnits::mV);
  drive_P(800,8);
  turn_P(-80,5);
  drive_P(550,8);
  wait(200,msec);
  drive_P(-700,8);
  turn_P(20,5);
  drive_P(700,8);
  wait(200,msec);
  drive_P(-700,8);
  turn_P(-120,5);
  drive_P(2000,8);
  */
 /*
   drive_P(-1000);
  drive_P(430);
  turn_P(-90);
  drive_P(-320);
  Intake.spin(forward,12000,vex::voltageUnits::mV);
  wait(500,msec);
  drive_P(320);
  Intake.spin(forward,0,vex::voltageUnits::mV);
  turn_P(-135);
  clamp1.set(true);
  clamp2.set(true);
  drive_P(-2200);
  clamp1.set(false);
  clamp2.set(false);
  turn_P(-135);
  Intake.spin(forward,12000,vex::voltageUnits::mV);
  drive_P(800);
  wait(250,msec);
  turn_P(-80);
  drive_P(500);
  wait(500,msec);
  drive_P(-700);
  turn_P(20);
  wait(250,msec);
  drive_P(950);
  wait(500,msec);
  drive_P(-950);
  wait(250,msec);
  turn_P(-90);
  drive_P(2000);
  */
}
void Skills (){
  Intake.spin(forward,12000,voltageUnits::mV);
  wait(500,msec);
  drive_P(650,10);
  turn_P(90,10);
  wait(200,msec);
  clamp1.set(true);
  clamp2.set(true);
  drive_P(-1500,13);
  clamp1.set(false);
  clamp2.set(false);
  wait(200,msec);
  turn_P(180,10);
  drive_P(1300,13);
  wait(200,msec); 
  drive_P(-500,10);
  turn_P(90,10);
  drive_P(900,10);
  turn_P(-20,10);
  drive_P(1100,13);
  drive_P(-1000,13);
  turn_P(-150,10);
  drive_P(1500,25);
  wait(500,msec);
  drive_P(-1000,13);
  turn_P(-130,10);
  drive_P(1000,13);
  turn_P(-10,10);
  drive_P(-1500,20);


  /*
  Intake.spin(forward,12000,voltageUnits::mV);
  wait(500,msec);
  drive_P(600);
  turn_P(90,10);
  clamp1.set(true);
  clamp2.set(true);
  drive_P(-1500);
  clamp1.set(false);
  clamp2.set(false);
  wait(500,msec);
  drive_P(-500);
  turn_P(-45,10);
  drive_P(-500);
  clamp1.set(true);
  clamp2.set(true);
  drive_P(-400);
  L1.spin(reverse,12000,vex::voltageUnits::mV);
  L2.spin(reverse,12000,vex::voltageUnits::mV);
  L3.spin(reverse,12000,vex::voltageUnits::mV);
  R1.spin(reverse,12000,vex::voltageUnits::mV);
  R2.spin(reverse,12000,vex::voltageUnits::mV);
  R3.spin(fwd,12000,vex::voltageUnits::mV);
  wait(500,msec);
  drive_P(300);
  turn_P(190);
  Intake.spin(forward,12000,vex::voltageUnits::mV);
  drive_P(750);
  wait(100,msec);
  drive_P(700);
  turn_P(90);
  drive_P(2000);
  turn_P(150);
  drive_P(800);
  turn_P(30);
  drive_P(1500);
  turn_P(-120);
  drive_P(-500);
  clamp1.set(true);
  clamp2.set(true);
  drive_P(1000);
  turn_P(-140);
  drive_P(-2000);
  clamp1.set(false);
  clamp2.set(false);
  turn_P(186);
  drive_P(750);
  wait(100,msec);
  drive_P(500);
  wait(100,msec);
  turn_P(-90);
  drive_P(750);
  turn_P(-150);
  drive_P(800);
  turn_P(-45);
  drive_P(1000);
  wait(100,msec);
  drive_P(-250);
  turn_P(135);
  drive_P(1000);
  wait(100,msec);
  drive_P(-2000);
  clamp1.set(true);
  clamp2.set(true);
  drive_P(1750);
  turn_P(-135);
  drive_P(-5000);
  turn_P(-90);
  drive_P(-2000);
  clamp1.set(false);
  clamp2.set(false);
  turn_P(45);
  drive_P(1000);
  turn_P(-45);
  drive_P(1000);
  turn_P(-90);
  drive_P(1400);
  turn_P(135);
  drive_P(750);
  turn_P(-90);
  drive_P(1000);
  Intake.spin(forward,0,vex::voltageUnits::mV);
  drive_P(-400);
  turn_P(186);
  drive_P(-400);
  clamp1.set(true);
  clamp2.set(true);
  drive_P(500);
  */
}
/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              Autonomous Task                              */
/*                                                                           */
/*  This task is used to control your robot during the autonomous phase of   */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/
//Catalog of numbers (DO NOT DELETE(very Important)) +/- 1 deg on avg:
//180 deg, cP: 100 (174-176 on rotation) actual value (for 180):186
//170 deg, cP:100 (166 deg on rotation) actual value (for 170):174.5
//160 deg, cP:100 (158 deg on rotation) actual value (for 160):163.5
//150 deg, cP:99
//140 deg, cP:95
//135 deg, cP:92.5
//130 deg, cP:90
//120 deg, cP:85
//110 deg, cP:80
//100 deg, cP:73
//090 deg, cP:68
//080 deg, cP:64
//070 deg, cP:57
//060 deg, cP:52
//050 deg, cP:46
//045 deg, cP:45
//040 deg, cP:43
//030 deg, cP:39
//020 deg, cP:32
//010 deg, cP:28; negative : 27.5
void runAuto(int i){
  switch(i)
  {
    case 1: 
      Red1();
    break;
    case 2:
      Blue1();
    break;
    case 3:
      Red2();
    break;
    case 4:
      Blue2();
    break;
    case 5:
      Skills();
    break;
  }
}

void autonomous(void) {
  runAuto(autonum);
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              User Control Task                            */
/*                                                                           */
/*  This task is used to control your robot during the user control phase of */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

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
  //vex::task wallTask(wallStakeTask);
  while (1) {
    int rightPower = Controller1.Axis2.position()*120;
    int leftPower = Controller1.Axis3.position()*120;
    L1.spin(forward, leftPower,voltageUnits::mV);
    L2.spin(forward, leftPower,voltageUnits::mV);
    L3.spin(forward, leftPower,voltageUnits::mV);
    R1.spin(forward, rightPower,voltageUnits::mV);
    R2.spin(forward, rightPower,voltageUnits::mV);
    R3.spin(forward, rightPower,voltageUnits::mV);
    if(Controller1.ButtonR1.pressing())
    {
      Intake.spin(forward, 12000,vex::voltageUnits::mV);
    }
    else if(Controller1.ButtonR2.pressing())
    {
      Intake.spin(reverse, 12000,vex::voltageUnits::mV);
    }else
    {
      Intake.spin(reverse, 0,vex::voltageUnits::mV);
    }
    if (Controller1.ButtonL1.pressing() and !prevValue2) {
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
     if (Controller1.ButtonL2.pressing() and !prevValue) {
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
    if(Brain.Screen.pressing())
    {
     wait(500,msec);
     autonum++;
    }
    if(autonum > 5)
    {
      autonum = 1;
    }
    wait(100, msec);
  }
}
