#include "vex.h"
#include <iostream>
#include <cstdlib>
using namespace vex;
#include "robot-config.hpp"

static void MoveDrivetrain() {
    double TempSpeed = 1;
    double DriveSpeed = (Controller.Axis3.position()) * 0.99 * TempSpeed;
    double TurnSpeed = (Controller.Axis1.position()) * 0.99 * TempSpeed;
    LeftDrive.setVelocity((DriveSpeed + TurnSpeed), pct);
    RightDrive.setVelocity((DriveSpeed - TurnSpeed), pct);

    if (((FL.temperature(temperatureUnits::fahrenheit)) >= 131) || ((ML.temperature(temperatureUnits::fahrenheit)) >= 131) || 
    ((BL.temperature(temperatureUnits::fahrenheit)) >= 131) || ((FR.temperature(temperatureUnits::fahrenheit)) >= 131) || 
    ((MR.temperature(temperatureUnits::fahrenheit)) >= 131) || ((BR.temperature(temperatureUnits::fahrenheit)) >= 131)) {
      TempSpeed = 0.13;
    }
    else if (((FL.temperature(temperatureUnits::fahrenheit)) > 90) || ((ML.temperature(temperatureUnits::fahrenheit)) > 90) || 
    ((BL.temperature(temperatureUnits::fahrenheit)) > 90) || ((FR.temperature(temperatureUnits::fahrenheit)) > 90) || 
    ((MR.temperature(temperatureUnits::fahrenheit)) > 90) || ((BR.temperature(temperatureUnits::fahrenheit)) > 90)) {
      TempSpeed = 0.66;
    }
    else {
      TempSpeed = 1;
    }

    LeftDrive.spin(forward);
    RightDrive.spin(forward);

}

int Speed = 0;

static void MoveIntake() {
    
    int IntakeSpeed;

    if(Controller.ButtonY.pressing()) {
      if(Speed == 1) {
      Speed = 0;
    }
      else {
        Speed = 1;
      }
    }

    if(Speed == 0) {
      IntakeSpeed = 35;
    }
    
    else {
      IntakeSpeed = 80;
    }
    
    Intake1.setVelocity(99, pct);
    Intake2.setVelocity(IntakeSpeed, pct);
    
    if(Controller.ButtonL2.pressing()) {
      Intake.spin(forward);
    }
    
    if(Controller.ButtonL1.pressing()) {
      Intake.spin(reverse);
    }

    if(Controller.ButtonX.pressing()) {
      Intake.stop();
    }
}

int doinker = 0;

static void MoveDoinker() {
  
  if(Controller.ButtonA.pressing()) {
    D.set(false);
  }

  if(Controller.ButtonB.pressing()){
    D.set(true);
  }
}

static void MoveMogo() {
    
    if(Controller.ButtonR2.pressing()) {
      P.set(true);
    }
     
    else if(Controller.ButtonR1.pressing()) {
      P.set(false);
    }
}

static void InitializeWallStake() {
  WallStake.setPosition(0, deg);
}


static void MoveWallStake() {
  int wallstakepressed;
  WallStake.setVelocity(50, pct);
  if(Controller.ButtonUp.pressing() || Controller.ButtonDown.pressing()) {
    wallstakepressed = 1;
  }
  else {
    wallstakepressed = 0;
  }
  if(0 <= WallStake.position(deg) && WallStake.position(deg) < 170) {
    while(Controller.ButtonUp.pressing()) {
      if(WallStake.position(deg) >= 170) {
        WallStake.spin(reverse);
      }
      WallStake.spin(forward);
    }

    while(Controller.ButtonDown.pressing()) {
      WallStake.spin(reverse);
    }
  }
  while(wallstakepressed == 1) {
    WallStake.stop();
  }
}

void drivercontrol() {
    InitializeWallStake();
    while(true){
      MoveDrivetrain();
      MoveIntake();
      MoveDoinker();
      //MoveClaw();
      MoveMogo();
      MoveWallStake();

      wait(20, msec);
    }
}

void vexcodeInit();
