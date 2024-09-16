#include "vex.h"
#include "auton.h"
#include <iostream>
#include "robot-config.hpp"

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
    
    while(1) {
        lefterror = target - LeftDrive.position(turns) * 3.25 * M_PI * 0.75;
        leftd = (lefterror - plefterror) * 40;
        lefttotal = lefterror * kp + lefti * ki - leftd * kd;

        LeftDrive.spin(forward, lefttotal, pct);

        if(fabs(lefterror) < 8) {
            lefti += lefterror / 40;
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

static void turnPID(std::string direction, double kp, double ki, double kd, double target) {
    double error = target;
    double perror = error;
    double d;
    double i = 0;
    if (target > 180) target -= 360;
    if (target < -180) target += 360;
    double total  = 0;
    double restedstates = 0;
    double stalledstates = 0;
    Inertial.resetRotation();

    while(restedstates<5&&stalledstates<50) {
        error = target - Inertial.rotation(degrees);
        if (fabs(error) < 30) i += error;
        if (fabs(error) < 0.1) i = 0;
        if (fabs(error - perror) < 0.005) stalledstates++;
        else stalledstates = 0;
        double out = kp * error + ki * i + kd * (error - perror);
        
        
        perror = error;

        wait(20,msec);

        if ((fabs(target - Inertial.rotation(degrees))) < 2){
            restedstates++;
        }

        else restedstates = 0;
       


        if(direction == "right") {
            LeftDrive.spin(forward, total, pct);
            RightDrive.spin(reverse, total, pct);
        }

        if(direction == "left") {
            LeftDrive.spin(reverse, total, pct);
            RightDrive.spin(forward, total, pct);
        }

       

        wait(20, msec);
    }

    LeftDrive.stop(brake);
    RightDrive.stop(brake);

    wait(100, msec);
}


static void drive(std::string direction, double target) {
    if(direction == "forward") {            
        drivePID(1, 0.02, 0.95, target);
    }

    if(direction == "reverse") {
        drivePID(1, 0.02, 0.95, -target);
    }
}

static void turn(std::string direction, double target) {
    if(direction == "right") {
        turnPID("right", 0, 0, 0, target);
    }

    if(direction == "left") {
        turnPID("left", 0, 0, 0, target);
    }
}

void AWPRed() {

    turn("right", 90);

}

void AWPBlue() {

}

void Red() {

}

void Blue() {

}