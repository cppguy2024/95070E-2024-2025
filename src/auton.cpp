#include "vex.h"
#include "auton.h"
#include <iostream>
#include "robot-config.hpp"

static void drivePID(double kp, double ki, double kd, double target) {
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
        leftd = (lefterror - plefterror) * 40;
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

        LeftDrive.spin(forward, lefttotal, pct);

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
        rightd = (righterror - prighterror) * 40;
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

        RightDrive.spin(forward, righttotal, pct);

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

static double geterror(double target) {
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

static void turnPID(double kp, double ki, double kd, double target) {
    double error = geterror(target);
    double perror = error;
    double d = 0;
    double i = 0;
    double reali = 0;
    double ptotal  = 0;
    double total = 0;
    double saturation = 0;
    double sign = 0;
    //Bohan is the sigmest rookie this year asdfasdfasdfasdfasdfasdfasdf
    while(fabs(error) > 1) {
        error = geterror(target);
        d = (error - perror) * 40;
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
        
        LeftDrive.spin(forward, total, pct);
        RightDrive.spin(reverse, total, pct);

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
        drivePID(1.5, 0.01, 0.9, target);
    }

    if(direction == "reverse") {
        drivePID(1, 0.02, 0.95, -target);
    }
}

static void turn(double target) {
    turnPID(0.4, 0.001, 0, target);
}

void AWPRed() {

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
    drive("forward", 84);
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
   
}

void Blue() {
    P.set(false);
    D.set(false);
    Intake.setVelocity(99, pct);
    drive("reverse", 27.5);
    P.set(true);
    Intake.spin(forward);
    drive("reverse", 3);
    turn(225);
    drive("forward", 19);
    turn(255);
    drive("forward", 9);
    wait(1000, msec);
    drive("reverse", 24);
    turn(292.5);
    drive("forward", 20);
    turn(55);
    drive("forward", 33);
    D.set(true);
    drive("reverse", 15);
    D.set(false);
    turn(55);
    drive("forward", 15);
    wait(4000, msec);
    Intake.stop();
}

void GoalRushRed() {

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
