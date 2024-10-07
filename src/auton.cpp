#include "vex.h"
#include "auton.h"
#include <iostream>
#include "robot-config.hpp"

static int() {

}

static void drivePID(double kp, double ki, double kd, double target) {
    double lefterror = target;
    double plefterror = lefterror;
    double leftd = 0;
    double lefti = 0;
    double lefttotal = 0;

    double righterror = target;
    double prighterror = righterror;
    double rightd = 0;
    double righti = 0;
    double righttotal = 0;
    
    LeftDrive.resetPosition();
    RightDrive.resetPosition();
    
    while((fabs(lefterror) + fabs(righterror)) / 2 > 0.5) {
        lefterror = target - LeftDrive.position(turns) * 3.25 * M_PI * 0.75;
        leftd = (lefterror - plefterror) * 40;
        lefttotal = lefterror * kp + lefti * ki - leftd * kd;

        LeftDrive.spin(forward, lefttotal, pct);

        if(fabs(lefterror) < 10) {
            lefti += lefterror / 50;
        }
        
        righterror = target - RightDrive.position(turns) * 3.25 * M_PI * 0.75;
        rightd = (righterror - prighterror) * 40;
        righttotal = righterror * kp + righti * ki - rightd * kd;

        RightDrive.spin(forward, righttotal, pct);

        if(fabs(righterror) < 8) {
            righti += righterror / 40;
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
    double total  = 0;
    //Bohan is the sigmest rookie this year asdfasdfasdfasdfasdfasdfasdf
    while(fabs(error) > 1) {
        error = geterror(target);
        d = (error - perror) * 40;
        total = error * kp + i * ki - d * kd;
        
        LeftDrive.spin(forward, total, pct);
        RightDrive.spin(reverse, total, pct);

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
    turn(90);
    drive("forward", 22);
    wait(1000, msec);
    turn(270);
    drive("forward", 33);
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
    drive("forward", 20);
    turn(255);
    drive("forward", 10);
    wait(1000, msec);
    drive("reverse", 24);
    turn(292.5);
    drive("forward", 20);
    turn(51.5);
    drive("forward", 33);
    D.set(true);
    drive("reverse", 15);
    D.set(false);
    turn(55);
    drive("forward", 15);
    wait(4000, msec);
    Intake.stop();
}
