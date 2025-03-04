#include "vex.h"
using namespace vex;

void redAWPStates () {
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
}

void blueAWPStates () {
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

void redPositiveQ() {
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

void bluePositiveQ(){

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

void redNegativeQ(){
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

void blueNegativeQ() {
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
  
}

void skills () {
  vex::task stall(intakeTask);
  //tune this
  moveWallStake(450,wallstake,12000);
  wait(200,msec);
  moveWallStake(-450,wallstake,12000);
  clamp1.set(true);
  clamp2.set(true);
  drive_P(-800,10,120);
  clamp1.set(false);
  clamp2.set(false);
  wait(400,msec);
  turn_P(-120,10);
  intake.isRunningIntake = true;
  Intake.spin(forward,12000,vex::voltageUnits::mV);
  drive_P(700,10,120);
  correction(400,-55,10);
  drive_P(1000,20,90);
  correction(400,-37,10);
}

void runAuto(int i){
  switch(i) {
    case 1:
      redAWPStates();
      break;
    case 2: 
      redPositiveQ();
      break;
    case 3:
      bluePositiveQ();
      break;
    case 4:
      redNegativeQ();
      break;
    case 5:
      blueNegativeQ();
      break;
    case 6:
      skills();
      break;
  }
}