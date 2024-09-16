#include "vex.h"
#include "robot-config.hpp"
using namespace vex;
using signature = vision::signature;
using code = vision::code;


brain Brain;
controller Controller;


motor FL = motor(PORT4, ratio6_1, true);
motor ML = motor(PORT5, ratio6_1, true);
motor BL = motor(PORT2, ratio6_1, true);
motor_group LeftDrive = motor_group(FL, ML, BL);
motor FR = motor(PORT11, ratio6_1, false);
motor MR = motor(PORT12, ratio6_1, false);
motor BR = motor(PORT13, ratio6_1, false);
motor_group RightDrive = motor_group(FR, MR, BR);


digital_out P = digital_out(Brain.ThreeWirePort.H);


motor Intake1 = motor(PORT3, ratio18_1, true);
motor Intake2 = motor(PORT10, ratio6_1, true);
motor_group Intake = motor_group(Intake1, Intake2);

//motor Claw = motor(PORT, ratio18_1, false);

inertial Inertial = inertial(PORT20);

void vexcodeInit(void);
