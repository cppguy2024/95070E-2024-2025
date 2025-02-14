#include "vex.h"
#include "robot-config.hpp"
using namespace vex;
using signature = vision::signature;
using code = vision::code;


brain Brain;
controller Controller;


motor FL = motor(PORT10, ratio6_1, true);
motor ML = motor(PORT11, ratio6_1, true);
motor BL = motor(PORT13, ratio6_1, false);
motor_group LeftDrive = motor_group(FL, ML, BL);
motor FR = motor(PORT1, ratio6_1, false);
motor MR = motor(PORT19, ratio6_1, false);
motor BR = motor(PORT20, ratio6_1, true);
motor_group RightDrive = motor_group(FR, MR, BR);


digital_out P = digital_out(Brain.ThreeWirePort.H);
digital_out D = digital_out(Brain.ThreeWirePort.G);

motor Intake1 = motor(PORT15, ratio6_1, true);
motor Intake2 = motor(PORT18, ratio6_1, false);
motor_group Intake = motor_group(Intake1,Intake2);



rotation Rotation = rotation(PORT12);
motor WallStake = motor(PORT8, ratio36_1, false);

inertial Inertial = inertial(PORT9);

void vexcodeInit(void);
