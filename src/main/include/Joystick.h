#ifndef JOYSTICK_H_
#define JOYSTICK_H_

#include <frc/Joystick.h>
#include "frc/smartdashboard/Smartdashboard.h"
#include <cmath>
#include "Common.h"

// Velocity in inches per second
#define SLOW_MAX_VEL (40.0)
#define FAST_MAX_VEL (180.00)
#define FAST_MAX_ROTATION_VEL (360.0)
#define SLOW_MAX_ROTATION_VEL (120.0)

// filter to prevent zero vel error
#define kExperimentalCMDFilter (0.75)

class Robot;
class Joystick
{

public:
    Joystick(Robot *pRobot);
    void LocalReset(void);
    void Service(void);
    void StopAll(void);
    double GetAxis2() { return axis2; };
    double GetAxis5() { return axis5; };
    double GetAxis0() { return axis0; };
    double GetAxis1() { return axis1; };
    double GetXCmd();
    double GetYCmd();
    double GetZCmd();
    double GetZeroVelRobotCentricSwitch() { return button4; };
    double GetFieldCentricSwitch() { return button5; };
    double GetResetGyroButton() { return button3; };
    double GetXYScaleFactor();
    double GetZScaleFactor();
    double GetGyroEnabled() { return button1; };
    double Shape(double x, double m);
    void UpdateDash();

private:
    Robot *mainRobot;

    frc::Joystick *flightController;

    double axis0;
    double axis1;

    double axis2;
    double axis5;

    double psidot;
    double xdot;
    double ydot;
    // gyro on/off switch
    double button1;
    // velocity fast/slow switch
    double button2;
    // reset gyro angle button
    double button3;
    // field centric to robot centric buttons
    double button4;
    double button5;

    double joystickX;
    double joystickY;
    double joystickZ;
};

#endif // JOYSTICK_H_