#include "vex.h"
#include "auton.h"
#include <iostream>
#include "robot-config.hpp"

static int getSign(double input){
  if (input > 0){
    return 1;
  }
  else if (input < 0){
    return -1;
  }
  else{
    return 0;
  }
}

static void drivePID(double kp, double ki, double kd, double minspeed, double target) {
    double lefterror = target;
    double plefterror = lefterror;
    double leftd = 0;
    double lefti = 0;
    double reallefti = 0;
    double plefttotal = 0;
    double lefttotal = 0;
    double leftsaturation = 0;
    double leftsign = 0;

    double righterror = target;
    double prighterror = righterror;
    double rightd = 0;
    double righti = 0;
    double realrighti = 0;
    double prighttotal = 0;
    double righttotal = 0;
    double rightsaturation = 0;
    double rightsign = 0;
    
    LeftDrive.resetPosition();
    RightDrive.resetPosition();
    
    while((fabs(lefterror) + fabs(righterror)) / 2 > 0.5) {
        lefterror = target - LeftDrive.position(turns) * 3.25 * M_PI * 0.75;
        leftd = (lefterror - plefterror) * 50;
        lefttotal = lefterror * kp + lefti * ki - leftd * kd;
        
        
        if(plefttotal == lefttotal) {
            leftsaturation = 0;
        }
        else {
            leftsaturation = 1;
        }
        if((plefttotal < 0) && (lefttotal < 0)) {
            leftsign = -1;
        }
        else {
            leftsign = 1;
        }

        if(fabs(lefttotal) < minspeed) {
            LeftDrive.spin(forward, getSign(lefterror) * minspeed, pct);
        }
        else {
            LeftDrive.spin(forward, lefttotal, pct);
        }

        
        if((leftsaturation == 1) || (leftsign == 1 )) {
            lefti = 0;
        }
        else {
           lefti = reallefti; 
        }
        if(fabs(lefterror) < 10) {
                lefti += lefterror / 50;
        }
        
        
        righterror = target - RightDrive.position(turns) * 3.25 * M_PI * 0.75;
        rightd = (righterror - prighterror) * 50;
        righttotal = righterror * kp + righti * ki - rightd * kd;
        if(prighttotal == righttotal) {
            rightsaturation = 0;
        }
        else {
            rightsaturation = 1;
        }
        if((plefttotal < 0) && (lefttotal < 0)) {
            rightsign = -1;
        }
        else {
            rightsign = 1;
        }

        
        if(fabs(righttotal) < minspeed) {
            RightDrive.spin(forward, getSign(righterror) * minspeed, pct);
        }
        else {
            RightDrive.spin(forward, righttotal, pct);
        }

        
        if((rightsaturation == 1) || (rightsign == 1 )) {
            righti = 0;
        }
        else {
          righti = realrighti;  
        }
        if(fabs(righterror) < 10) {
                realrighti += righterror / 50;
        }

        plefterror = lefterror;
        prighterror = righterror;

        
        wait(20, msec);
    }

    LeftDrive.stop(brake);
    RightDrive.stop(brake);

    wait(100, msec);
}

static double getError(double target) {
    if((std::max(target, Inertial.heading()) - std::min(target, Inertial.heading())) > 180) {
        if(std::min(target, Inertial.heading()) == target) {
            return (360 - std::max(target, Inertial.heading()) + std::min(target, Inertial.heading()));
        }
        else {
            return -(360 - std::max(target, Inertial.heading()) + std::min(target, Inertial.heading()));
        }
    }
    else {
        return (target - Inertial.heading());
    }
}

static void turnPID(double kp, double ki, double kd, double minspeed, double target) {
    double error = getError(target);
    double perror = error;
    double d = 0;
    double i = 0;
    double reali = 0;
    double ptotal  = 0;
    double total = 0;
    double saturation = 0;
    double sign = 0;
    //Bohan is the sigmest rookie this year asdfasdfasdfasdfasdfasdfasdf
    while(fabs(error) > 2 || d > 3) {
        error = getError(target);
        d = (error - perror) * 50;
        ptotal = total;
        total = error * kp + i * ki - d * kd;
        
        
        if(ptotal == total) {
            saturation = 0;
        }
        else {
            saturation = 1;
        }
        if((ptotal < 0) && (total < 0)) {
            sign = -1;
        }
        else {
            sign = 1;
        }
        
        
        if(fabs(total) < minspeed) {
            LeftDrive.spin(forward, getSign(error) * minspeed, pct);
            RightDrive.spin(forward, getSign(error) * minspeed, pct);
        }
        else {
            LeftDrive.spin(forward, total, pct);
            RightDrive.spin(forward, total, pct);
        }

        
        if((saturation == 1) && (sign == 1)) {
            i = 0;
        }
        else {
            i = reali;
        }
        if(fabs(error) < 20) {
            i += error/50;
        }
        
        perror = error;

        wait(20, msec);
    }

    LeftDrive.stop(brake);
    RightDrive.stop(brake);

    wait(100, msec);
}

static void drive(std::string direction, double target) {
    if(direction == "forward") {            
        drivePID(0, 0, 0, 40, target);
    }

    if(direction == "reverse") {
        drivePID(0, 0, 0, 40, -target);
    }
}

static void turn(double target) {
    turnPID(0, 0, 0, 15, target);
}

/*void AWPRed() {
    P.set(false);
    D.set(false);
    Intake.setVelocity(99, pct);
    drive("reverse", 27.5);
    P.set(true);
    Intake.spin(forward);
    drive("reverse", 2);
    turn(90);
    drive("forward", 24);
    turn(0);
    P.set(false);
    drive("forward", 24);
    turn(270);
    drive("forward", 87);
    turn(330);
    drive("reverse", 22);
    P.set(true);
    turn(285);
    drive("forward", 22);
    wait(1000, msec);
    turn(80);
    drive("forward", 34);
}

void AWPBlue() {
    P.set(false);
    D.set(false);
    Intake.setVelocity(99, pct);
    drive("reverse", 27.5);
    P.set(true);
    Intake.spin(forward);
    drive("reverse", 2);
    turn(270);
    drive("forward", 24);
    turn(0);
    P.set(false);
    drive("forward", 24);
    turn(90);
    drive("forward", 87);
    turn(30);
    drive("reverse", 22);
    P.set(true);
    turn(75);
    drive("forward", 22);
    wait(1000, msec);
    turn(280);
    drive("forward", 34);
}

void Red() {
    P.set(false);
    D.set(false);
    Intake.setVelocity(99, pct);
    drive("reverse", 27.5);
    P.set(true);
    Intake.spin(forward);
    drive("reverse", 3);
    turn(120);
    drive("forward", 19);
    turn(105);
    drive("forward", 8.25);
    wait(1000, msec);
    drive("reverse", 24);
    turn(67.5);
    drive("forward", 20);
    turn(305);
    drive("forward", 33);
    D.set(true);
    drive("reverse", 15);
    D.set(false);
    turn(295);
    drive("forward", 15);
    wait(4000, msec);
    Intake.stop();
}

void Blue() {
    P.set(false);
    D.set(false);
    Intake.setVelocity(99, pct);
    drive("reverse", 27.5);
    P.set(true);
    Intake.spin(forward);
    drive("reverse", 3);
    turn(230);
    drive("forward", 19);
    turn(255);
    drive("forward", 8.25);
    wait(1000, msec);
    drive("reverse", 24);
    turn(287.5);
    drive("forward", 20);
    wait(2000, msec);
    Intake.stop();
    /*turn(55);
    drive("forward", 33);
    D.set(true);
    drive("reverse", 15);
    D.set(false);
    turn(65);
    drive("forward", 15);
    wait(4000, msec);
    Intake.stop();
}

void GoalRushRed() {
    P.set(false);
    D.set(false);
    Intake.setVelocity(99, pct);
    drive("reverse", 30);
    turn(330);
    drive("reverse", 19);
    P.set(true);
    Intake.spin(forward);
}

void GoalRushBlue() {
    P.set(false);
    D.set(false);
    Intake.setVelocity(99, pct);
    drive("reverse", 30);
    turn(30);
    drive("reverse", 19); 
    P.set(true);
    Intake.spin(forward);
}

void AutonSkills() {
    P.set(false);
    D.set(false);
    Intake.setVelocity(99,pct);
    Intake.spin(forward);
    wait(1000, msec);
    drive("forward", 13.5);
    turn(90);
    drive("reverse", 23);
    P.set(true);
    wait(1000, msec);
    turn(270);
    drive("forward", 18);
    wait(1000, msec);
    drive("forward", 14);
    turn(35);
    drive("reverse", 10);
    wait(1000, msec);
    Intake.spin(reverse);
    wait(250, msec);
    Intake.stop();
    P.set(false);
    drive("forward", 8.75);
    wait(1000, msec);
    turn(270);
    drive("reverse", 18.25);
    wait(500, msec);
    drive("reverse", 18.25);
    wait(500, msec);
    drive("reverse", 18.25);
    wait(500, msec);
    drive("reverse", 20.25);
    wait(1000, msec);
    Intake.spin(forward);
    P.set(true);
    turn(90);
    drive("forward", 18);
    wait(1000, msec);
    drive("forward", 14);
    turn(325);
    drive("reverse", 14);
    P.set(false);
}*/

void AWPRed() {

}

void AWPBlue() {

}

void Red() {

}

void Blue() {

}

void GoalRushRed() {

}

void GoalRushBlue() {

}

void AutonSkills() {

}
