// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <string>
#include <frc/TimedRobot.h>
#include <frc/smartdashboard/SendableChooser.h>
#include "Joystick.h"
#include "Drivetrain.h"

class Robot : public frc::TimedRobot
{
public:
    Robot();
    Drivetrain *drivetrain;
    Joystick *joystick;

    void RobotInit() override;
    void RobotPeriodic() override;
    void AutonomousInit() override;
    void AutonomousPeriodic() override;
    void TeleopInit() override;
    void TeleopPeriodic() override;
    void DisabledInit() override;
    void DisabledPeriodic() override;
    void TestInit() override;
    void TestPeriodic() override;
    void SimulationInit() override;
    void SimulationPeriodic() override;
    void ServiceDash();
    void LocalReset();
    double GetLoopCount() { return count; };
    static double Wrap(double val);
    static double Limit(double min, double max, double val);

private:
    double joystickDriveCmd;
    double joystickSteerCmd;
    double count;

    frc::SendableChooser<std::string> m_chooser;
    const std::string kAutoNameDefault = "Default";
    const std::string kAutoNameCustom = "My Auto";
    std::string m_autoSelected;
};