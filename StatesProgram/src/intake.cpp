#include "vex.h"
using namespace vex;


RIntake intake;
bool isRecovering = false;


int intakeStallMonitor() {
  while (true) {
      double velocity = Intake.velocity(percent);

      if (intake.isRunningIntake && !isRecovering && fabs(velocity) < 1 && fabs(12) > 0) {
          Brain.Screen.printAt(10, 40, "Stall detected! Running recovery...");
          isRecovering = true;

          // Reverse the motor briefly
          Intake.spin(directionType::rev, 12000, vex::voltageUnits::mV);
          wait(300, msec);

          // Restore original speed
          Intake.spin(directionType::fwd, 12000, vex::voltageUnits::mV);
          wait(500, msec); // Wait before allowing another stall detection

          isRecovering = false; // Re-enable stall detection
      }
      wait(100, msec); // Check every 100ms
  }
  return 0;
}

int intakeTask() {
  while(true) {
    intakeStallMonitor();
  }
  return 0;
}