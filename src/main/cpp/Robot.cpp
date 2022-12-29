// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <fmt/core.h>
#include "frc/RobotController.h"
#include <cmath>
#include <units/time.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include "Robot.h"
#include "Common.h"

/**
 * @brief Construct a new TimedRobot object
 *
 */
Robot::Robot() : TimedRobot(units::second_t LOOPTIME)
{
  drivetrain = new Drivetrain(this);
  joystick = new Joystick(this);
  count = 0;
}
/**
 * @brief Initialize auto and reset states
 *
 */
void Robot::RobotInit()
{
  drivetrain->LocalReset();
  joystick->LocalReset();

  m_chooser.SetDefaultOption(kAutoNameDefault, kAutoNameDefault);
  m_chooser.AddOption(kAutoNameCustom, kAutoNameCustom);
  frc::SmartDashboard::PutData("Auto Modes", &m_chooser);
}

/**
 * This function is called every robot packet, no matter the mode. Use
 * this for items like diagnostics that you want ran during disabled,
 * autonomous, teleoperated and test.
 *
 * <p> This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */
void Robot::RobotPeriodic() {}

/**
 * This autonomous (along with the chooser code above) shows how to select
 * between different autonomous modes using the dashboard. The sendable chooser
 * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
 * remove all of the chooser code and uncomment the GetString line to get the
 * auto name from the text box below the Gyro.
 *
 * You can add additional auto modes by adding additional comparisons to the
 * if-else structure below with additional strings. If using the SendableChooser
 * make sure to add them to the chooser code above as well.
 */
void Robot::AutonomousInit()
{
  m_autoSelected = m_chooser.GetSelected();
  // m_autoSelected = SmartDashboard::GetString("Auto Selector",
  //     kAutoNameDefault);
  fmt::print("Auto selected: {}\n", m_autoSelected);

  if (m_autoSelected == kAutoNameCustom)
  {
    // Custom Auto goes here
  }
  else
  {
    // Default Auto goes here
  }
}

void Robot::AutonomousPeriodic()
{
  if (m_autoSelected == kAutoNameCustom)
  {
    // Custom Auto goes here
  }
  else
  {
    // Default Auto goes here
  }
}
/**
 * @brief Initialize subsystems before enabling
 *
 */
void Robot::TeleopInit()
{
  LocalReset();
}
/**
 * @brief Reset subsystems and variables
 *
 */
void Robot::LocalReset()
{
  joystick->LocalReset();
  drivetrain->LocalReset();
  count = 0;
  joystickDriveCmd = 0.0;
  joystickSteerCmd = 0.0;
}
/**
 * @brief Perform robot functions
 *
 */
void Robot::TeleopPeriodic()
{
  double xdot, ydot, psidot;
  count++;

  // Get sensor data
  drivetrain->Analyze();
  joystick->Service();

  // joystick cmds
  xdot = joystick->GetXCmd();
  ydot = joystick->GetYCmd();
  psidot = joystick->GetZCmd();

  if (joystick->GetResetGyroButton())
  {
    drivetrain->GyroReset();
  }

  if (joystick->GetFieldCentricSwitch() == 1 && joystick->GetZeroVelRobotCentricSwitch() == 0)
  {
    drivetrain->ToFieldCoordinates(&xdot, &ydot);
    drivetrain->SetZeroVelDebugModeOff();
  }
  else if (joystick->GetFieldCentricSwitch() == 0 && joystick->GetZeroVelRobotCentricSwitch() == 0)
  {
    drivetrain->SetZeroVelDebugModeOn();
  }
  else
  {
    drivetrain->SetZeroVelDebugModeOff();
  }
  if (joystick->GetGyroEnabled() > 0.5)
  {
    drivetrain->EnableGyro();
  }
  else
  {
    drivetrain->DisableGyro();
  }
  ServiceDash();
  // control robot positioning
  drivetrain->DriveControl(xdot, ydot, psidot);
}
/**
 * @brief Update SmartDashboard
 *
 */
void Robot::ServiceDash()
{
  joystick->UpdateDash();
  drivetrain->UpdateDash();
  frc::SmartDashboard::PutNumber("joystick drive cmd", joystickDriveCmd);
  frc::SmartDashboard::PutNumber("joystick steer cmd", joystickSteerCmd);
  frc::SmartDashboard::PutNumber("Loop count", count);
  frc::SmartDashboard::PutNumber("CAN pct utilization", (frc::RobotController::GetCANStatus().percentBusUtilization * 100));
  frc::SmartDashboard::PutNumber("Loop count", count);
}
void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}

void Robot::TestPeriodic() {}

void Robot::SimulationInit() {}

void Robot::SimulationPeriodic() {}

/**
 * @brief Wrap value in degrees
 *
 * @param val value in degrees
 * @return double wrapped value in degrees
 */
double Robot::Wrap(double val)
{
  double rVal = val * (M_PI / 180);
  double rWrap = atan2(sin(rVal), cos(rVal));
  return rWrap * (180 / M_PI);
}
/**
 * @brief Limit the value between a range
 *
 * @param min minimum value
 * @param max maximum value
 * @param val value to compare
 * @return double
 */
double Robot::Limit(double min, double max, double val)
{
  if (val > max)
  {
    return max;
  }
  if (val < min)
  {
    return min;
  }
  return val;
}

#ifndef RUNNING_FRC_TESTS
int main()
{
  return frc::StartRobot<Robot>();
  // double v1c, v2c, v3c, v4c, t1c, t2c, t3c, t4c;
  // SwerveKinematics(0.5, 0.5, 0.3, v1c, v2c, v3c, v4c, t1c, t2c, t3c, t4c);
  // printf("%f %f %f %f %f %f %f %f \n", v1c, v2c, v3c, v4c, t1c, t2c, t3c, t4c);
}
#endif
