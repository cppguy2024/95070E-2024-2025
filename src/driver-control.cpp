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

      else if(Speed == 0) {
        Speed = 1;
      }

      else {
        Speed = 1;
      }
    }

    if(Speed == 1) {
      IntakeSpeed = 70;
    }

    else if(Speed == 0) {
      IntakeSpeed = 35;
    }

    else {
      IntakeSpeed = 70;
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


static void doinker(){
  if(Controller.ButtonA.pressing()) {
      Doinker.set(true);
    }
  if(Controller.ButtonB.pressing()) {
      Doinker.set(false);
    }
}

/*static void MoveClaw() {
    
    Claw.setVelocity(100, pct);
  
    while(Controller.ButtonA.pressing()) {
      Claw.spin(forward);
    }

    while(Controller.ButtonB.pressing()) {
      Claw.spin(reverse);
    }

    Claw.stop();
}*/

static void MoveMogo() {
    
    if(Controller.ButtonR2.pressing()) {
      P.set(true);
    }
     
    if(Controller.ButtonR1.pressing()) {
      P.set(false);
    }
}

void drivercontrol() {
    while(true){
      MoveDrivetrain();
      MoveIntake();
      //MoveClaw();
      MoveMogo();
      doinker();

      wait(20, msec);
    }
}

void vexcodeInit();