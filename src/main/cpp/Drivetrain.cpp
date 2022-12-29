#include <frc/smartdashboard/SmartDashboard.h>
#include <cmath>
#include "Common.h"
#include "Drivetrain.h"
#include "Robot.h"
/**
 * @brief Construct a new Drivetrain object
 *
 * @param pRobot pointer to the main robot class
 */
Drivetrain::Drivetrain(Robot *pRobot)
{
    mainRobot = pRobot;

    frontLeftModule = new SwerveModule(mainRobot, FRONT_LEFT_STEER, FRONT_LEFT_DRIVE, FRONT_LEFT_ABSOLUTE_ENCODER, 4);
    frontRightModule = new SwerveModule(mainRobot, FRONT_RIGHT_STEER, FRONT_RIGHT_DRIVE, FRONT_RIGHT_ABSOLUTE_ENCODER, 1);
    backLeftModule = new SwerveModule(mainRobot, BACK_LEFT_STEER, BACK_LEFT_DRIVE, BACK_LEFT_ABSOLUTE_ENCODER, 3);
    backRightModule = new SwerveModule(mainRobot, BACK_RIGHT_STEER, BACK_RIGHT_DRIVE, BACK_RIGHT_ABSOLUTE_ENCODER, 2);

    gyro = new frc::ADXRS450_Gyro(frc::SPI::kOnboardCS0);
    gyro->Calibrate();
    LocalReset();
}
/**
 * @brief Reset variables and swerve submodules
 *
 */
void Drivetrain::LocalReset()
{
    zeroVelDebugMode = false;

    frontLeftModule->LocalReset();
    frontRightModule->LocalReset();
    backLeftModule->LocalReset();
    backRightModule->LocalReset();

    gyro->Reset();
    gyroReading = gyro->GetAngle();
    gyroVel = gyro->GetRate();

    gyroLastReading = gyroReading;

    r_vCmd = 0.0;
    r_vErr = 0.0;
    r_vErrIntegrator = 0.0;
    r_cmd = 0.0;
    gyroVel = 0.0;
    r_ff = 0.0;

    xf = 0.0;
    yf = 0.0;
    zf = 0.0;
}
/**
 * @brief Computes swerve velocity and rotational commands
 *
 * @param xdot forward velocity from joystick
 * @param ydot right velocity from joystick
 * @param psidot rotation rate around z from joystick. coming in as degrees, converted to radians.
 *
 * @param v1c velocity command for 1st module in inches/sec
 * @param v2c velocity command for 2nd module in inches/sec
 * @param v3c velocity command for 3rd module in inches/sec
 * @param v4c velocity command for 4th module in inches/sec
 *
 * @param t1c rotational command for 1st module in deg/sec
 * @param t2c rotational command for 2nd module in deg/sec
 * @param t3c rotational command for 3rd module in deg/sec
 * @param t4c rotational command for 4th module in deg/sec
 */
void Drivetrain::SwerveKinematics(double xdot, double ydot, double psidot,
                                  double &v1c, double &v2c, double &v3c, double &v4c,
                                  double &t1c, double &t2c, double &t3c, double &t4c)
{
    double v1x, v1y, v2x, v2y, v3x, v3y, v4x, v4y;
    psidot = (psidot * M_PI / 180);

    v1x = xdot - (psidot * (P1Y - CY));
    v1y = ydot + (psidot * (P1X - CX));

    v2x = xdot - (psidot * (P2Y - CY));
    v2y = ydot + (psidot * (P2X - CX));

    v3x = xdot - (psidot * (P3Y - CY));
    v3y = ydot + (psidot * (P3X - CX));

    v4x = xdot - (psidot * (P4Y - CY));
    v4y = ydot + (psidot * (P4X - CX));

    v1c = sqrt((v1x * v1x) + (v1y * v1y));
    v2c = sqrt((v2x * v2x) + (v2y * v2y));
    v3c = sqrt((v3x * v3x) + (v3y * v3y));
    v4c = sqrt((v4x * v4x) + (v4y * v4y));

    t1c = (180 / M_PI) * atan2(v1y, v1x) + P1Offset;
    t2c = (180 / M_PI) * atan2(v2y, v2x) + P2Offset;
    t3c = (180 / M_PI) * atan2(v3y, v3x) + P3Offset;
    t4c = (180 / M_PI) * atan2(v4y, v4x) + P4Offset;

    //   if (v1y==0 && v1x==0){
    //       t1c = P1Offset - PStart;
    //   }
    //    if (v2y==0 && v2x==0){
    //       t2c = P2Offset + PStart;
    //   }
    //    if (v3y==0 && v3x==0){
    //       t3c = P3Offset - PStart;
    //   }
    //    if (v4y==0 && v4x==0){
    //       t4c = P4Offset + PStart;
    //   }

    t1c = Robot::Wrap(t1c);
    t2c = Robot::Wrap(t2c);
    t3c = Robot::Wrap(t3c);
    t4c = Robot::Wrap(t4c);
}
/**
 * @brief Read sensor values
 *
 */
void Drivetrain::Analyze()
{
    frontLeftModule->Analyze();
    frontRightModule->Analyze();
    backRightModule->Analyze();
    backLeftModule->Analyze();

    gyroReading = gyro->GetAngle();
    gyroVel = (gyroReading - gyroLastReading) / LOOPTIME;
    gyroVel = Robot::Limit(-GYRO_MAX_VEL, GYRO_MAX_VEL, gyroVel);
    gyroLastReading = gyroReading;
    gyroReading = Robot::Wrap(gyroReading);
}
/**
 * @brief Reset gyro zero
 *
 */
void Drivetrain::GyroReset()
{
    gyro->Reset();
    gyroLastReading = gyro->GetAngle();
}
/**
 * @brief Control drivetrain
 *
 * @param xdot joystick x
 * @param ydot joystick y
 * @param psidot joystick z
 */
void Drivetrain::DriveControl(double xdot, double ydot, double psidot)
{
    // gyro heading loop
    // TODO fix temp_x
    gyroVel = GetGyroVel();
    r_ff = psidot;
    r_vErr = psidot - gyroVel;
    r_vErrIntegrator += r_vErr * LOOPTIME;
    double temp_x;
    temp_x = (mainRobot->joystick->GetZScaleFactor() * R_kR) / R_kV;
    r_vErrIntegrator = Robot::Limit(-temp_x, temp_x, r_vErrIntegrator);
    r_cmd = (R_kV / R_kR) * (R_TAU * r_vErr + r_vErrIntegrator) + (R_FF * r_ff);

    if (!gyroEnabled)
    {
        r_cmd = psidot;
        r_vErrIntegrator = 0.0;
    }

    double v1c, v2c, v3c, v4c, t1c, t2c, t3c, t4c;
    double xdotFil, ydotFil, psidotFil;
    CmdModel(xdot, ydot, r_cmd, &xdotFil, &ydotFil, &psidotFil, 10, 10, 10);
    SwerveKinematics(xdotFil, ydotFil, psidotFil, v1c, v2c, v3c, v4c, t1c, t2c, t3c, t4c);

    if (zeroVelDebugMode)
    {
        v1c = 0.0;
        v2c = 0.0;
        v3c = 0.0;
        v4c = 0.0;
    }

    frontRightModule->DriveControl(v1c, t1c);
    backRightModule->DriveControl(v2c, t2c);
    backLeftModule->DriveControl(v3c, t3c);
    frontLeftModule->DriveControl(v4c, t4c);
}
/**
 * @brief Joystick command model filter
 *
 * @param x joystick xdot
 * @param y joystick ydot
 * @param z joystick psidot
 * @param pxout xdot out
 * @param pyout ydot out
 * @param pzout psidot out
 * @param kx x constant
 * @param ky y constant
 * @param kz z constant
 */
void Drivetrain::CmdModel(double x, double y, double z,
                          double *pxout, double *pyout, double *pzout,
                          double kx, double ky, double kz)
{
    double xfdot, yfdot, zfdot;

    xfdot = (x - xf) * kx;
    yfdot = (y - yf) * ky;
    zfdot = (z - zf) * kz;

    xf = xf + xfdot * LOOPTIME;
    yf = yf + yfdot * LOOPTIME;
    zf = zf + zfdot * LOOPTIME;

    *pxout = xf;
    *pyout = yf;
    *pzout = zf;
    // *pxout = x;
    // *pyout = y;
    // *pzout = z;
}
/**
 * @brief Convert robot centric coordinates to field centric coordinates
 *
 * @param *xdot pointer to joystick xdot
 * @param *ydot pointer to joystick ydot
 */
void Drivetrain::ToFieldCoordinates(double *xdot, double *ydot)
{

    double vNorth, vEast, theta;
    vNorth = *xdot;
    vEast = *ydot;
    // field coordinates conversion
    theta = GetGyroReading() * (M_PI / 180.0);
    theta = theta + GetGyroVel() * (M_PI / 180.0) * 0.15;
    *xdot = cos(theta) * vNorth + sin(theta) * vEast;
    *ydot = -sin(theta) * vNorth + cos(theta) * vEast;
}
/**
 * @brief Update SmartDashboard
 *
 */
void Drivetrain::UpdateDash()
{
    frontRightModule->UpdateDash();
    frontLeftModule->UpdateDash();
    backLeftModule->UpdateDash();
    backRightModule->UpdateDash();

    frc::SmartDashboard::PutNumber("gyro wrapped angle", gyroReading);
    frc::SmartDashboard::PutNumber("gyro vel", gyroVel);
}
/**
 * @brief Stop submodules
 *
 */
void Drivetrain::StopAll()
{
    frontLeftModule->StopAll();
    frontRightModule->StopAll();
    backLeftModule->StopAll();
    backRightModule->StopAll();
}