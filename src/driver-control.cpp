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

int intake = 0;
static void MoveIntake() {
    Intake.setVelocity(80, pct);
    if(Controller.ButtonL1.pressing()) {
        intake += 1;

        if(intake%2 == 1) {
            Intake.spin(forward);
        }

        if(intake%2 == 0) {
            Intake.stop();
        }
        wait(150, msec);
    }
    
    if(Controller.ButtonL2.pressing()) {
        Intake.spin(reverse);
    }
}


int doinker = 0;
static void MoveDoinker(){
  if(Controller.ButtonR1.pressing()) {
    doinker += 1;
    if(doinker%2 == 1) {
        D.set(true);
    }

    if(doinker%2 == 0) {
        D.set(false);
    }
    wait(150, msec);
  }
}



int mogo = 0;
static void MoveMogo() {
    if(Controller.ButtonR2.pressing()) {
        mogo += 1;
        if(mogo%2 == 1) {
            P.set(true);
            wait(150, msec);
        }

        if(mogo%2 == 0) {
            P.set(false);
            wait(150, msec);
        }
    }
}

static void InitializeWallStake() {
  Rotation.setPosition(0, deg);
}

static void MoveWallStake() {
  double position = Rotation.angle(deg);
  double error = 20 - position;
  
    if(Controller.ButtonUp.pressing()) {
        WallStake.setVelocity(70, pct);
        WallStake.spin(forward);
    }
  
    else if(Controller.ButtonDown.pressing()) {
        WallStake.setVelocity(70, pct);
        WallStake.spin(reverse);
    }

    else if(Controller.ButtonB.pressing()) {
        if(fabs(error) > 3) {
            WallStake.spin(forward, 0.2 * error, pct);
        }
    }

    else {
    WallStake.stop();
    }
}


void drivercontrol() {
    InitializeWallStake();
    Intake.setVelocity(99, pct);
    while(true){
      MoveDrivetrain();
      MoveIntake();
      MoveDoinker();
      MoveMogo();
      MoveWallStake();

      wait(20, msec);
    }
}

void vexcodeInit();
