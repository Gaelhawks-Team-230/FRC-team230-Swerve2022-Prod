#include <cmath>
#include "Common.h"
#include "Joystick.h"
/**
 * @brief Construct a new Joystick:: Joystick object
 *
 * @param pRobot pointer to the main robot class
 */
Joystick::Joystick(Robot *pRobot)
{
    mainRobot = pRobot;
    flightController = new frc::Joystick(0);
}
/**
 * @brief Resets local variables
 *
 */
void Joystick::LocalReset()
{
    axis0 = 0.0;
    axis1 = 0.0;
    axis2 = 0.0;
    axis5 = 0.0;
    button1 = 0.0;
    button2 = 0.0;
    button3 = 0.0;
    button4 = 0.0;
    button5 = 0.0;
    xdot = 0.0;
    ydot = 0.0;
    psidot = 0.0;
    joystickY = 0.0;
    joystickX = 0.0;
    joystickZ = 0.0;
}
/**
 * @brief Get updated joystick state
 *
 */
void Joystick::Service()
{
    axis0 = flightController->GetRawAxis(0);
    axis1 = flightController->GetRawAxis(1);
    axis2 = flightController->GetRawAxis(2);
    axis5 = flightController->GetRawAxis(5);
    button1 = flightController->GetRawButton(1);
    button2 = flightController->GetRawButton(2);
    button3 = flightController->GetRawButton(3);
    button4 = flightController->GetRawButton(4);
    button5 = flightController->GetRawButton(5);
}
/**
 * @brief Updating dashboard
 *
 */
void Joystick::UpdateDash()
{
    frc::SmartDashboard::PutNumber("Axis 0: ", axis0);
    frc::SmartDashboard::PutNumber("Axis 1: ", axis1);
    frc::SmartDashboard::PutNumber("Axis 2: ", axis2);
    frc::SmartDashboard::PutNumber("Axis 5: ", axis5);
    frc::SmartDashboard::PutNumber("Button 1: ", button1);
    frc::SmartDashboard::PutNumber("Button 2: ", button2);
    frc::SmartDashboard::PutNumber("Button 3: ", button3);
    frc::SmartDashboard::PutNumber("Button 4: ", button4);
    frc::SmartDashboard::PutNumber("Button 5: ", button5);

    frc::SmartDashboard::PutNumber("joystickX", joystickX);
    frc::SmartDashboard::PutNumber("joystickY", joystickY);
}
/**
 * @brief Shape x axis and apply scale factor
 *
 * @return double x velocity cmd
 */
double Joystick::GetXCmd()
{
    joystickX = axis1 * -1;
    joystickX = Shape(joystickX, 0.6);
    xdot = joystickX * GetXYScaleFactor();
    return xdot;
}
/**
 * @brief Shape z axis and apply scale factor
 *
 * @return double z velocity cmd
 */
double Joystick::GetZCmd()
{
    joystickZ = axis5;
    joystickZ = Shape(joystickZ, 1.0);
    psidot = joystickZ * GetZScaleFactor();
    return psidot;
}
/**
 * @brief Shape y axis and apply scale factor
 *
 * @return double y velocity cmd
 */
double Joystick::GetYCmd()
{
    joystickY = axis0;
    joystickY = Shape(joystickY, 0.6);
    ydot = joystickY * GetXYScaleFactor();
    return ydot;
}
/**
 * @brief Get max velocity
 *
 * @return double max velocity
 */
double Joystick::GetXYScaleFactor()
{
    if (button2 == 0)
    {
        return SLOW_MAX_VEL;
    }
    else
    {
        return FAST_MAX_VEL;
    }
}
/**
 * @brief Get max rotational velocity
 *
 * @return double max velocity
 */
double Joystick::GetZScaleFactor()
{
    if (button2 == 0)
    {
        return SLOW_MAX_ROTATION_VEL;
    }
    else
    {
        return FAST_MAX_ROTATION_VEL;
    }
}
/**
 * @brief Apply an exponential curve to joystick values
 *
 * @return double shaped value
 */
#define SIGN(x) (((x) > 0) ? (1) : (-1))
double Joystick::Shape(double x, double m)
{
    double db = 0.05;
    double xdb = fmax((fabs(x) - db) / (1.0 - db), 0.0) * SIGN(x);
    double y = ((0.5 * m * fabs(xdb) - 0.5 * m + 1) * xdb);
    return y;
}
