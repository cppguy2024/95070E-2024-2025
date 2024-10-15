#include "vex.h"
#include "auton.h"
#include <iostream>
#include "robot-config.hpp"
#include <cmath>
#include <algorithm>

void Odometry::update(double deltaLeft, double deltaRight) {
    double deltaDistance = (deltaLeft + deltaRight) / 2.0;
    x += deltaDistance * cos(theta);
    y += deltaDistance * sin(theta);
    theta += (deltaRight - deltaLeft) / wheelBase; // Update heading
    theta = fmod(theta + 2 * M_PI, 2 * M_PI); // Normalize angle
}

double Odometry::getX() { return x; }
double Odometry::getY() { return y; }
double Odometry::getTheta() { return theta; }

// PID Implementation
PID::PID(double kP, double kI, double kD) 
    : kP(kP), kI(kI), kD(kD), lastError(0), integral(0) {}

double PID::compute(double setpoint, double actual) {
    double error = setpoint - actual;
    integral += error;

    // Anti-windup
    if (integral > integralLimit) integral = integralLimit;
    if (integral < -integralLimit) integral = -integralLimit;

    double derivative = error - lastError;
    lastError = error;

    return clamp(kP * error + kI * integral + kD * derivative, -maxSpeed, maxSpeed);
}

// Arc Movement Function
void moveArc(double targetX, double targetY, double radius) {
    
    double targetAngle = atan2(targetY - odometry.getY(), targetX - odometry.getX());
    double distanceToTarget = sqrt(pow(targetX - odometry.getX(), 2) + pow(targetY - odometry.getY(), 2));

    while (distanceToTarget > tolerance) {
        odometry.update(LeftDrive.position(rotationUnits::deg), RightDrive.position(rotationUnits::deg));
        // Calculate errors
        double errorX = targetX - odometry.getX();
        double errorY = targetY - odometry.getY();
        double errorTheta = targetAngle - odometry.getTheta();

        // Calculate PID outputs
        double outputX = pidX.compute(targetX, odometry.getX());
        double outputY = pidY.compute(targetY, odometry.getY());
        double outputTurn = pidTurn.compute(targetAngle, odometry.getTheta());

        // Calculate speeds
        double leftSpeed = outputY - outputX + outputTurn;
        double rightSpeed = outputY + outputX - outputTurn;

        // Set motor speeds
        LeftDrive.spin(vex::forward, clamp(leftSpeed, -maxSpeed, maxSpeed), percent);
        RightDrive.spin(vex::forward, clamp(rightSpeed, -maxSpeed, maxSpeed), percent);

        // Update distance to target
        distanceToTarget = sqrt(pow(targetX - odometry.getX(), 2) + pow(targetY - odometry.getY(), 2));
        task::sleep(20);
    }

    LeftDrive.stop();
    RightDrive.stop();
}

// Autonomous Function
void autonomous() {
    Inertial.calibrate(); // Calibrate the IMU
    wait(2000, msec); // Wait for calibration to finish

    // Example of moving in an arc
    moveArc(100, 100, 50); // Move to (100, 100) with a radius of 50 inches

}
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

        LeftDrive.spin(vex::forward, lefttotal, pct);

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

        RightDrive.spin(vex::forward, righttotal, pct);

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
        
        LeftDrive.spin(vex::forward, total, pct);
        RightDrive.spin(vex::reverse, total, pct);

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
    P.set(false);
    D.set(false);
    Intake.setVelocity(99, pct);
    drive("reverse", 27.5);
    P.set(true);
    Intake.spin(vex::forward);
    drive("reverse", 2);
    turn(90);
    drive("forward", 24);
    turn(0);
    P.set(false);
    drive("forward", 24);
    turn(270);
    drive("forward", 84);
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
    Intake.spin(vex::forward);
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
    P.set(false);
    D.set(false);
    Intake.setVelocity(99, pct);
    drive("reverse", 27.5);
    P.set(true);
    Intake.spin(vex::forward);
    drive("reverse", 3);
    turn(135);
    drive("forward", 19);
    turn(105);
    drive("forward", 9);
    wait(1000, msec);
    drive("reverse", 24);
    turn(67.5);
    drive("forward", 20);
    turn(305);
    drive("forward", 33);
    D.set(true);
    drive("reverse", 15);
    D.set(false);
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
    Intake.spin(vex::forward);
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
    Intake.spin(vex::forward);
}

void GoalRushBlue() {

    moveArc(50,10,30);
    P.set(false);
    D.set(false);
    Intake.setVelocity(99, pct);
    drive("reverse", 30);
    turn(30);
    drive("reverse", 19);
    P.set(true);
    Intake.spin(vex::forward);
}

void autonskills(){
    Odometry::update;
     
    P.set(false);
    D.set(false);
    Intake.setVelocity(99,pct);
    Intake.spin(vex::forward);
    wait(3,seconds);
    turn(225);
    drive("reverse",26.83281573);
    wait(1,seconds);
    turn(0); 
    drive("forward",36);
    wait(1,seconds);
    turn(270);
    drive("forward",24);
    wait(500,msec);
    turn(0);
        wait(500,msec);
    drive("forward",36);
    wait(1,seconds);
    turn(45);
    wait(1,seconds);
    drive("forward",67.882250994);
        wait(500,msec);

    turn(315);
        wait(500,msec);

    drive("forward",24);
        wait(500,msec);

    turn(45);
        wait(500,msec);

    drive("forward",36);
        wait(500,msec);

    turn(225);
        wait(500,msec);

    drive("forward",120);
 

 
}
