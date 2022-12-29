#ifndef SWERVEMODULE_H_
#define SWERVEMODULE_H_

#include <frc/DigitalInput.h>
#include "frc/smartdashboard/Smartdashboard.h"
#include "ctre/Phoenix.h"
#include "ctre/phoenix/motorcontrol/can/WPI_TalonFX.h"
#include "ctre/phoenix/sensors/CANCoder.h"
#include <cmath>
#include <frc/DutyCycle.h>
#include "Common.h"

#define STEER_kP (10.0)
#define STEER_kV (10.0)
#define STEER_kR (3000.0)
#define STEER_TAU (0.05)

#define DRIVE_kV (10.0)
#define DRIVE_kR (200.0)
#define DRIVE_TAU (0.05)
#define kAV (0.0185)


#define COUNTS_100MS (1.0 / 100.0)
#define MILLISECONDS_SEC (100.0 / 0.1)
#define REV_COUNTS (1.0 / 2048.0)
#define RAD_REV (2.0 * M_PI)
#define WHEEL_CIRCUMFERENCE (2.00)
#define GEAR_RATIO (1.0 / 6.75)

#define VELOCITY_MEAS_WINDOW (32.0)
#define DRIVE_DEADBAND (0.001)
#define STEER_DEADBAND (0.001)

class Robot;

class SwerveModule
{

public:
    SwerveModule(Robot *pRobot, unsigned int steerFalconID, unsigned int driveFalconID, unsigned int absoluteEncoderID, unsigned int swerveID);
    void LocalReset();
    void UpdateDash();
    void StopAll();
    void StartingConfig();

    void SetDriveCmd(double cmd);
    void SetWheelCmd(double cmd);
    void Analyze();
    void DriveControl(double vel, double angle);

    double GetDriveVel() { return driveVel; };
    double GetSteerEncoderVel() { return steerEncoderVel; };
    double GetSteerEncoderAbsolutePosition() { return steerEncoderAbsolutePosition; };
    double GetVelocityScaleFactor() { return COUNTS_100MS * MILLISECONDS_SEC * REV_COUNTS * RAD_REV * WHEEL_CIRCUMFERENCE * GEAR_RATIO; };
    double GetPositionVelocityScaleFactor() { return (1.0 / LOOPTIME) * REV_COUNTS * RAD_REV * WHEEL_CIRCUMFERENCE * GEAR_RATIO; };

private:
    Robot *mainRobot;
    unsigned int moduleID;
    ctre::phoenix::motorcontrol::can::WPI_TalonFX *steer;
    ctre::phoenix::motorcontrol::can::WPI_TalonFX *drive;
    ctre::phoenix::sensors::CANCoder *steerEncoder;

    double driveLastPosition;
    double driveCurrentPosition;
    double driveRaw;
    double driveOffset;
    double drivePosition;
    double driveVel;
    double driveMotorVel;
    double drivePosVel;

    double steerMotorPosition;
    double steerMotorVel;
    double steerEncoderPosition;
    double steerEncoderAbsolutePosition;
    double steerEncoderVel;

    double steer_pErr;
    double steer_vCmd;
    double steer_vErr;
    double steer_vErrIntegrator;
    double steer_cmd;

    bool rev;

    double drive_pErr;
    double drive_vCmd;
    double drive_vErr;
    double drive_vErrIntegrator;
    double drive_cmd;
};
#endif