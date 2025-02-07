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
        righterror = target - RightDrive.position(turns) * 3.25 * M_PI * 0.75;
        
        leftd = (lefterror - plefterror) * 40;
        rightd = (righterror - prighterror) * 40;
        
        lefttotal = lefterror * kp + lefti * ki - leftd * kd;
        righttotal = righterror * kp + righti * ki - rightd * kd;

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

        LeftDrive.spin(forward, lefttotal, pct);
        RightDrive.spin(forward, righttotal, pct);

        if((leftsaturation == 1) || (leftsign == 1 )) {
            lefti = 0;
        }
        else {
           lefti = reallefti; 
        }
        if(fabs(lefterror) < 10) {
            reallefti += lefterror / 50;
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

static void turnPID(double kp, double ki, double kd, double tolerance, double target) {
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
    while(fabs(error) > tolerance || (d > 3)) {
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

static void ramdrive(std::string direction, double target) {
    if(direction == "forward") {            
        drivePID(2.5, 0.001, 0.75, target);
    }

    if(direction == "reverse") {
        drivePID(2.5, 0.001, 0.75, -target);
    }
}

static void drive(std::string direction, double target) {
    if(direction == "forward") {            
        drivePID(1.8, 0.001, 0.75, target);
    }

    if(direction == "reverse") {
        drivePID(1.8, 0.001, 0.75, -target);
    }
}

static void slowdrive(std::string direction, double target) {
    if(direction == "forward") {            
        drivePID(1.3, 0.001, 0.75, target);
    }

    if(direction == "reverse") {
        drivePID(1.3, 0.001, 0.75, -target);
    }
}

static void turn(double target) {
    turnPID(0.4, 0.002, 0, 1, target);
}

void AWPRed() {
    Inertial.setHeading(307.54028, deg);
    WallStake.spin(forward, 75, pct);
    wait(1000, msec);
    WallStake.stop();
    drive("reverse", 3);
    turn(350);
    wait(100, msec);
    WallStake.spin(reverse, 99, pct);
    wait(500, msec);
    WallStake.stop();
    wait(50, msec);
    slowdrive("reverse", 32.5);
    P.set(true);
    turn(138);
    Intake.spin(forward, 99, pct);
    drive("forward", 22);
    turn(113);
    drive("forward", 8);
    wait(500,msec);
    drive("reverse", 24);
    turn(77);
    drive("forward", 17);
    wait(1000, msec);
    turn(260);
    drive("forward", 26);
    Intake.stop();
}

void AWPBlue() {
    Inertial.setHeading(52.45972, deg);
    WallStake.spin(forward, 75, pct);
    wait(1000, msec);
    WallStake.stop();
    drive("reverse", 3);
    turn(10);
    wait(100, msec);
    WallStake.spin(reverse, 99, pct);
    wait(500, msec);
    WallStake.stop();
    wait(50, msec);
    slowdrive("reverse", 32.5);
    P.set(true);
    turn(222);
    Intake.spin(forward, 99, pct);
    drive("forward", 22);
    turn(247);
    drive("forward", 8);
    wait(500,msec);
    drive("reverse", 24);
    turn(283);
    drive("forward", 17);
    wait(1000, msec);
    turn(100);
    drive("forward", 26);
    Intake.stop();
}

void Red() {

}

void Blue() {
    
}

void GoalRushRedQ() {
    Inertial.setHeading(347.3, deg);
    drive("forward", 36);
    WallStake.spin(forward, 50, pct);
    wait(1200, msec);
    WallStake.stop();
    wait(50, msec);
    WallStake.spin(reverse, 99, pct);
    wait(600, msec);
    WallStake.stop();
    turn(28);
    drive("forward", 10);
    Intake.spin(forward, 99, pct);
    wait(600, msec);
    Intake.stop();
    turn(35);
    slowdrive("reverse", 26);
    P.set(true);
    Intake.spin(forward, 99, pct);
    wait(500, msec);
    turn(270);
    drive("forward", 12);
    D.set(true);
    wait(200,msec);
    
}

void GoalRushBlueQ() {
    Inertial.setHeading(12.7, deg);
    drive("forward", 36);
    WallStake.spin(forward, 60, pct);
    wait(1200, msec);
    WallStake.stop();
    turn(332);
    drive("forward", 10);
    Intake.spin(forward, 99, pct);
    wait(50, msec);
    WallStake.spin(reverse, 99, pct);
    wait(500, msec);
    WallStake.stop();
    Intake.stop();
    turn(325);
    slowdrive("reverse", 26);
    P.set(true);
    Intake.spin(forward, 99, pct);
    wait(500, msec);
    turn(90);
    drive("forward", 12);
}

void GoalRushRedE() {
    Intake.spin(forward, 99, pct);
    drive("forward", 39);
    D.set(true);
    wait(350,msec);
    Intake.stop();
    slowdrive("reverse", 15);
    D.set(false);
    turn(80);
    slowdrive("reverse", 15);
    P.set(true);
    Intake.spin(forward, 99, pct);
    turn(170);
    slowdrive("forward", 26);
    turn(90);
    slowdrive("forward", 35);
    turn(105);
    slowdrive("forward", 5);
    wait(1500,msec);
    turn(170);
    D.set(true);
    turn(140);
    wait(20,msec);
    D.set(false);
    turn(170);  
   
}

void GoalRushBlueE() {
    Intake.spin(forward, 99, pct);
    drive("forward", 39);
    wait(050, msec);
    Intake.stop();
    D.set(true);
    wait(350, msec);
    slowdrive("reverse", 14);
    D.set(false);
    turn(250);
    wait(250,msec);
    slowdrive("reverse", 34);
    P.set(true);
    Intake.spin(forward, 99, pct);
    turn(200);
    drive("forward",32);
    turn(230);
    drive("forward",2);
    

}

void AutonSkills() {
    Inertial.setHeading(0, deg);
    Intake.spin(forward, 99 ,pct );
    wait(500, msec);
    drive("forward", 15);
    turn(90);
    slowdrive("reverse", 24);
    P.set(true);
    turn(0);
    wait(50, msec);
    drive("forward", 24);
    turn(303);
    drive("forward", 37);
    turn(160);
    wait(50, msec);
    slowdrive("forward", 32);
    turn(180);
    slowdrive("forward", 22);
    wait(500, msec);
    turn(320);
    drive("forward", 15);
    wait(500, msec);
    turn(15);
    drive("reverse", 10);
    P.set(false);
    drive("forward", 9);
    wait(1000, msec);
    turn(270);
    slowdrive("reverse", 60);
    slowdrive("reverse", 20);
    slowdrive("reverse", 5);
    P.set(true);
    turn(0);
    Inertial.setHeading(0, deg);
    drive("forward", 24);
    turn(56.5);
    slowdrive("forward", 37);
    turn(200);
    slowdrive("forward", 26);
    turn(180);
    slowdrive("forward", 27);
    wait(500, msec);
    turn(40);
    drive("forward", 15);
    wait(500, msec);
    turn(330);
    drive("reverse", 10);
    P.set(false);
    drive("forward", 15);
    turn(0);
    drive("forward", 35);
    turn(315);
    drive("forward", 85);
    turn(260);
    slowdrive("reverse", 50);
    drive("forward", 60);
    turn(290);
    ramdrive("forward", 50);
    drive("reverse", 5);
}
