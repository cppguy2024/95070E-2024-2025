#include "vex.h"
#include "auton.h"
#include <iostream>
#include "robot-config.hpp"

static void drivePID(double kp, double ki, double kd, double tolerance, double target) {
    double lefterror = target;
    double plefterror = lefterror;
    double leftd = 0;
    double lefti = 0;
    double reallefti = 0;
    double plefttotal = 0;
    double lefttotal = 0;
    double leftsaturation = 0;
    double leftsign = 0;
    double leftcounter = 0;

    double righterror = target;
    double prighterror = righterror;
    double rightd = 0;
    double righti = 0;
    double realrighti = 0;
    double prighttotal = 0;
    double righttotal = 0;
    double rightsaturation = 0;
    double rightsign = 0;
    double rightcounter = 0;
    
    LeftDrive.resetPosition();
    RightDrive.resetPosition();
    
    while((fabs(lefterror) + fabs(righterror)) / 2 > tolerance) {
        lefterror = target - LeftDrive.position(turns) * 3.25 * M_PI * 0.75;
        leftd = (lefterror - plefterror) * 47.5;
        lefttotal = lefterror * kp + lefti * ki - leftd * kd;
        
        
        if((lefterror - plefterror) < 0.075) {
            leftcounter += 50;
        }
        if(leftcounter == 1250) {
            lefttotal = 0;
        }


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
        rightd = (righterror - prighterror) * 50;
        righttotal = righterror * kp + righti * ki - rightd * kd;
        
        
        if((righterror - prighterror) < 0.075) {
            rightcounter += 50;
        }
        if(rightcounter == 1250) {
            righttotal = 0;
        }
        
        
        if(prighttotal == righttotal) {
            rightsaturation = 0;
        }
        else {
            rightsaturation = 1;
        }
        if((prighttotal < 0) && (righttotal < 0)) {
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

static void turnPID(double kp, double ki, double kd, double tolerance, double target) {
    double error = getError(target);
    double perror = error;
    double d = 0;
    double i = 0;
    double reali = 0;
    double ptotal  = 0;
    double total = 0;
    double saturation = 0;
    double sign = 0;
    double counter = 0;
    //Bohan is the sigmest rookie this year asdfasdfasdfasdfasdfasdfasdf
    while(fabs(error) > tolerance || d > 3) {
        error = getError(target);
        d = (error - perror) * 50;
        ptotal = total;
        total = error * kp + i * ki - d * kd;

        if((error - perror) < 0.025) {
            counter += 50;
        }
        if(counter == 1250) {
            total = 0;
        }
        
        
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
        drivePID(2.95, 0.01, 0.05, 0.5, target);
    }

    if(direction == "reverse") {
        drivePID(2.95, 0.01, 0.05, 0.5, -target);
    }
}

static void slowdrive(std::string direction, double target) {
    if(direction == "forward") {            
        drivePID(1.97, 0.01, 0.05, 0.75, target);
    }

    if(direction == "reverse") {
        drivePID(1.97, 0.01, 0.05, 0.75, -target);
    }
}

static void progdrive(std::string direction, double target) {
    if(direction == "forward") {            
        drivePID(1.5, 0.01, 0.05, 0.5, target);
    }

    if(direction == "reverse") {
        drivePID(1.5, 0.01, 0.05, 0.5, -target);
    }
}

static void turn(double target) {
    turnPID(0.38, 0.001, 0.003, 2, target);
}

static void mogoturn(double target) {
    turnPID(0.34, 0.001, 0.003, 2, target);
}


void Red() {
    P.set(false);
    D.set(false);
    Intake.setVelocity(99, pct);
    slowdrive("reverse", 29.5);
    P.set(true);
    wait(50, msec);
    Intake.spin(forward);
    wait(500, msec);
    drive("reverse", 1);
    mogoturn(90);
    drive("forward", 24);
    mogoturn(197);
    drive("forward", 11.5);
    wait(1000, msec);
    mogoturn(110);
    drive("forward", 6.5);
    wait(2000, msec);
    mogoturn(100);
    drive("reverse", 40);
    Intake.stop();
}

void Blue() {
    P.set(false);
    D.set(false);
    Intake.setVelocity(99, pct);
    slowdrive("reverse", 29.5);
    P.set(true);
    wait(50, msec);
    Intake.spin(forward);
    wait(500, msec);
    drive("reverse", 1);
    mogoturn(270);
    drive("forward", 24);
    mogoturn(163);
    drive("forward", 11.5);
    wait(1000, msec);
    mogoturn(250);
    drive("forward", 6.5);
    wait(2000, msec);
    mogoturn(260);
    drive("reverse", 40);
    Intake.stop();
}

void GoalRushRed() {
    P.set(false);
    D.set(false);
    drive("reverse", 34.5);
    mogoturn(330);
    drive("reverse", 15);
    P.set(true);
    Intake.setVelocity(99, pct);
    Intake.spin(forward);
    mogoturn(10);
    Intake.stop();
    Intake1.spin(forward);
    drive("forward", 21);
    P.set(false);
    mogoturn(280);
    slowdrive("reverse", 25);
    Intake1.stop();
    P.set(true);
    Intake2.spin(forward);
    wait(1000,msec);
    /*mogoturn(330);
    slowdrive("reverse",14);*/
    Intake2.stop();
}

void GoalRushBlue() {
    P.set(false);
    D.set(false);
    drive("reverse", 34.5);
    mogoturn(30);
    drive("reverse", 15);
    P.set(true);
    Intake.setVelocity(99, pct);
    Intake.spin(forward);
    mogoturn(350);
    Intake.stop();
    Intake1.spin(forward);
    drive("forward", 21);
    P.set(false);
    mogoturn(80);
    slowdrive("reverse", 25);
    Intake1.stop();
    P.set(true);
    Intake2.spin(forward);
    wait(1000,msec);
    /*mogoturn(30);
    slowdrive("reverse", 14);*/
    Intake2.stop();
}


void AutonSkills() {
    /*P.set(false);
    D.set(false);
    Intake.setVelocity(99,pct);
    Intake.spin(forward);
    wait(1000, msec);
    drive("forward", 13.5);
    turn(90);
    slowdrive("reverse", 23);
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
    slowdrive("reverse", 37.5);
    wait(200, msec);
    slowdrive("reverse", 37.5);
    wait(1000, msec);
    Intake.spin(forward);
    P.set(true);
    turn(90);
    drive("forward", 20);
    wait(1000, msec);
    drive("forward", 14);
    turn(325);
    drive("reverse", 16);
    P.set(false);
    wait(500, msec);
    drive("forward", 20);*/
    P.set(false);
    D.set(false);
    Intake.setVelocity(99, pct);
    Intake.spin(forward);
    wait(1000, msec);
    drive("forward", 13.5);
    turn(90);
    progdrive("reverse", 23);
    P.set(true);
    wait(1000, msec);
    turn(270);
    drive("forward", 18);
    wait(1000, msec);
    drive("forward", 14);
    wait(1000, msec);
    drive("reverse", 10);
    turn(180);
    drive("forward", 10);
    wait(1000, msec);
    drive("reverse", 10);
    turn(45);
    drive("reverse", 17);
    wait(1000, msec);
    Intake.spin(reverse);
    wait(250, msec);
    Intake.stop();
    P.set(false);
    drive("forward", 9.75);
    wait(1000, msec);
    turn(270);
    progdrive("reverse", 37.5);
    wait(200, msec);
    progdrive("reverse", 37.5);
    wait(1000, msec);
    Intake.spin(forward);
    P.set(true);
    turn(90);
    drive("forward", 20);
    wait(1000, msec);
    drive("forward", 10);
    turn(325);
    drive("reverse", 14);
    P.set(false);
    turn(315);
    drive("reverse", 15);
    wait(1000,msec);
    drive("forward",135);
    turn(45);
}
