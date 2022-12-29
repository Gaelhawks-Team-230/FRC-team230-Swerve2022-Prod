#ifndef DRIVETRAIN_H_
#define DRIVETRAIN_H_

#include <frc/ADXRS450_Gyro.h>
#include "Common.h"
#include "SwerveModule.h"

// Measured in inches, used in code as inches
// Point 1 coordinates:
const double P1X = (24.75);
const double P1Y = (24.75);
// Point 2 coordinates:
const double P2X = (3.25);
const double P2Y = (24.75);
// Point 3 coordinates:
const double P3X = (3.25);
const double P3Y = (3.25);
// Point 4 coordinates:
const double P4X = (24.75);
const double P4Y = (3.25);
// Center coordinates:
// originally both 14
const double CX = (14.0);
const double CY = (14.0);
// Offset of theta for each module
const double P1Offset = 51.86;
const double P2Offset = -32.34;
const double P3Offset = -69.79;
const double P4Offset = 171.04;
const double PStart = 0;

#define GYRO_MAX_VEL (360.0) // degrees per second

#define R_kV (5.0)
#define R_kR (1.0)
// #define R_TAU (0.05)
#define R_TAU (0.075)
#define R_FF (0.0)

class Robot;

class Drivetrain
{

private:
    Robot *mainRobot;

    SwerveModule *frontLeftModule;
    SwerveModule *frontRightModule;
    SwerveModule *backLeftModule;
    SwerveModule *backRightModule;

    frc::ADXRS450_Gyro *gyro;
    double gyroReading;
    double gyroVel;
    double gyroLastReading;
    bool zeroVelDebugMode;
    bool gyroEnabled;

    double xf;
    double yf;
    double zf;

    // gyro stuff
    double r_vCmd;
    double r_vErr;
    double r_vErrIntegrator;
    double r_cmd;
    double r_ff;

public:
    Drivetrain(Robot *pRobot);
    void LocalReset(void);
    void StartingConfig(void);
    void StopAll(void);
    void UpdateDash(void);
    void DriveControl(double xdot, double ydot, double psidot);
    void Analyze(void);
    void GyroReset(void);
    double GetGyroReading(void) { return gyroReading; };
    double GetGyroVel(void) { return gyroVel; };
    void SetZeroVelDebugModeOn() { zeroVelDebugMode = true; };
    void SetZeroVelDebugModeOff() { zeroVelDebugMode = false; };
    void CmdModel(double x, double y, double z,
                  double *pxout, double *pyout, double *pzout,
                  double kx, double ky, double kz);

    void SwerveKinematics(double xdot, double ydot, double psidot,
                          double &v1c, double &v2c, double &v3c, double &v4c,
                          double &t1c, double &t2c, double &t3c, double &t4c);
    void ToFieldCoordinates(double *xdot, double *ydot);
    void EnableGyro(){ gyroEnabled=true;};
    void DisableGyro(){ gyroEnabled=false;};

};

#endif // DRIVETRAIN_H_