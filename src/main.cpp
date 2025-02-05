#include "vex.h"
#include "driver-control.hpp"
#include "auton.h"
#include <iostream>
#include <cstdlib>
#include "robot-config.hpp"
using namespace vex;
using signature = vision::signature;
using code = vision::code;


competition Competition;
bool inauton = false;


void vexcodeInit() {

}

int autons = 9;
int displayautons = 0;

void select(){
  while(1) {
    if (Controller.ButtonRight.pressing()) {
      displayautons++;
    }
    
    if (Controller.ButtonLeft.pressing()) {
      displayautons--;
    }

    if (Controller.ButtonA.pressing()) {
      wait(1000, msec);
      if (Controller.ButtonA.pressing()) {
        Controller.rumble(rumbleLong);
        break;
      }
    }
    
    if (displayautons > autons) {
      displayautons = 0;
    }

    if (displayautons < 0) {
      displayautons = autons;
    }

    if (displayautons == 0) {
        Controller.Screen.clearScreen();
        Controller.Screen.setCursor(1, 1);
        Controller.Screen.print("Please select an auton");
    }
    
    if (displayautons == 1){
        Controller.Screen.clearScreen();
        Controller.Screen.setCursor(1, 1);
        Controller.Screen.print("AWPRed");
    }
    
    if (displayautons == 2){
        Controller.Screen.clearScreen();
        Controller.Screen.setCursor(1, 1);
        Controller.Screen.print("AWPBlue");
    }
    
    if (displayautons == 3){
        Controller.Screen.clearScreen();
        Controller.Screen.setCursor(1, 1);
        Controller.Screen.print("Red");
    }
    
    if (displayautons == 4){
        Controller.Screen.clearScreen();
        Controller.Screen.setCursor(1, 1);
        Controller.Screen.print("Blue");
    }

    if (displayautons == 5){
        Controller.Screen.clearScreen();
        Controller.Screen.setCursor(1, 1);
        Controller.Screen.print("GoalRushRedQ");
    }

    if (displayautons == 6){
        Controller.Screen.clearScreen();
        Controller.Screen.setCursor(1, 1);
        Controller.Screen.print("GoalRushBlueQ");
    }

    if (displayautons == 7){
        Controller.Screen.clearScreen();
        Controller.Screen.setCursor(1, 1);
        Controller.Screen.print("GoalRushRedE");
    }

    if (displayautons == 8){
        Controller.Screen.clearScreen();
        Controller.Screen.setCursor(1, 1);
        Controller.Screen.print("GoalRushBlueE");
    }

    if (displayautons == 9){
        Controller.Screen.clearScreen();
        Controller.Screen.setCursor(1, 1);
        Controller.Screen.print("AutonSkills");
    }

  }
}

void pre_auton(void) {
  
  select();
  vexcodeInit();

}

void autonomous(void) {
  
  inauton = true;
  
  if(displayautons == 1) {
    AWPRed();
  }
  
  if(displayautons == 2) {
    AWPBlue();
  }
  
  if(displayautons == 3) {
    Red();
  }
  
  if(displayautons == 4) {
    Blue();
  }

  if(displayautons == 5) {
    GoalRushRedQ();
  }

  if(displayautons == 6) {
    GoalRushBlueQ();
  }

  if(displayautons == 7) {
    GoalRushRedE();
  }

  if(displayautons == 8) {
    GoalRushRedE();
  }

  if(displayautons == 9) {
    AutonSkills();
  }
  
  inauton=false;

}

int main() {
  Competition.drivercontrol(drivercontrol);
  Competition.autonomous(autonomous);

  pre_auton();
  
  while(1) {
    wait(20, msec);
  }
  void vexcodeInit(void);

}
