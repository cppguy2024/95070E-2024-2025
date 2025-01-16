#include "vex.h"
#include "auton.h"
#include <iostream>
#include "robot-config.hpp"

/*static void drivePID(double kp, double ki, double kd, double target) {
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

        if(fabs(error) < 16) {
            i += error/40;
        }

        wait(20, msec);
    }

    LeftDrive.stop(brake);
    RightDrive.stop(brake);

    wait(100, msec);
}*/

//lk, rk are P, ltaui, rtaui are I, ltaud, rtaud are D
void drivePID(double k, double taui, double taud, double tolerance, double target) {
    double setpoint = target;
    double leftprocessvalue = LeftDrive.position(turns) * M_PI * 3.25 * 0.75;
    double rightprocessvalue = RightDrive.position(turns) * M_PI * 3.25 * 0.75;
    
    double plefterror = 0;
    double lefterror = 0;
    double leftreset = 0;
    double lefttotal = 0;

    double prighterror = 0;
    double righterror = 0;
    double rightreset = 0;
    double righttotal = 0;

    while(((fabs(lefterror) + fabs(righterror))/2) > tolerance) {
        lefterror = setpoint - leftprocessvalue;
        righterror = setpoint - rightprocessvalue;
        
        leftreset += ((k/taui) * lefterror) * 50;
        rightreset += ((k/taui) * righterror) * 50;

        lefttotal = k * lefterror + leftreset + ((plefterror - lefterror) * (k/taud));
        righttotal = k * righterror + rightreset + ((prighterror - righterror) * (k/taud));

        LeftDrive.spin(forward, lefttotal, pct);
        RightDrive.spin(forward, righttotal, pct);

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

static void turn(double target) {
    turnPID(0.2, 0.02, 0, target);
}

void AWPRed() {
    
}

void AWPBlue() {

}

void Red() {

}

void Blue() {
    P.set(false);
    Intake.setVelocity(99, pct);
    drive("reverse", 22);
    P.set(true);
    Intake.spin(forward);
    drive("reverse", 8);
    /*turn(270);
    drive("forward", 24);
    turn(180);
    Intake.stop();
    P.set(false);*/
}
