/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       athar                                                     */
/*    Created:      2/17/2025, 5:06:33 PM                                     */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "vex.h"

using namespace vex;
int autonum = 4;

competition Competition;

void pre_auton(void) {

  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ...
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
  vex::task stall(intakeTask);
  int stall = 0;
  while (1) {
    
    intake.isRunningIntake = false;
    int rightPower = Controller1.Axis2.position()*120;
    int leftPower = Controller1.Axis3.position()*120;
    runDrivetrain(fwd,fwd,rightPower,leftPower);
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
