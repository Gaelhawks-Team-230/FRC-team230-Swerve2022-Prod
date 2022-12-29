#include "frc/RobotController.h"
#include "Common.h"
#include "SwerveModule.h"
#include "Robot.h"

using namespace std;

/**
 * @brief Constructor for a single swerve module
 *
 * @param pRobot main robot
 * @param steerFalconID CAN ID of steer falcon
 * @param driveFalconID CAN ID of drive falcon
 * @param absoluteEncoderID CAN ID of the CANCoder ID
 * @return SwerveModule
 */
SwerveModule::SwerveModule(Robot *pRobot, unsigned int steerFalconID, unsigned int driveFalconID, unsigned int absoluteEncoderID, unsigned int swerveID)
{

  mainRobot = pRobot;
  moduleID = swerveID;

  steer = new ctre::phoenix::motorcontrol::can::WPI_TalonFX(steerFalconID);
  drive = new ctre::phoenix::motorcontrol::can::WPI_TalonFX(driveFalconID);
  steerEncoder = new ctre::phoenix::sensors::CANCoder(absoluteEncoderID);

  StartingConfig();
  LocalReset();
}
/**
 * @brief Set Falcons to starting configuration
 *
 */
void SwerveModule::StartingConfig()
{
  steer->SetNeutralMode(NeutralMode::Brake);
  drive->SetNeutralMode(NeutralMode::Coast);
  drive->ConfigVelocityMeasurementPeriod(SensorVelocityMeasPeriod::Period_50Ms);
  drive->ConfigVelocityMeasurementWindow(VELOCITY_MEAS_WINDOW);
  
  drive->ConfigNeutralDeadband(DRIVE_DEADBAND);
  steer->ConfigNeutralDeadband(STEER_DEADBAND);
  
}
/**
 * @brief Reset all variabes to default
 *
 */
void SwerveModule::LocalReset()
{
  driveLastPosition = drive->GetSelectedSensorPosition();

  driveCurrentPosition = 0.0;
  drivePosVel = 0.0;
  rev = false;
  driveVel = 0.0;
}
/**
 * @brief Stop all motor functions and reset variables
 *
 */
void SwerveModule::StopAll()
{
  drive->StopMotor();
  steer->StopMotor();
  LocalReset();
}
/**
 * @brief Set drive falcon cmd
 *
 * @param cmd -1 to 1 falcon cmd
 * @note cmd is negated as motor direction is wrong
 */
void SwerveModule::SetDriveCmd(double cmd)
{
  drive->Set(-cmd);
}
/**
 * @brief Set steer falcon cmd
 *
 * @param cmd -1 to 1 falcon cmd
 * @note cmd is negated as motor direction is wrong
 */
void SwerveModule::SetWheelCmd(double cmd)
{
  steer->Set(-cmd);
}
/**
 * @brief Update dashboard with swerve module data
 *
 */
void SwerveModule::UpdateDash()
{
  string moduleName;
  moduleName = "Module " + to_string(moduleID) + ": ";

  frc::SmartDashboard::PutNumber(moduleName + "Drive raw position: ", drivePosition);
  frc::SmartDashboard::PutNumber(moduleName + "Drive velocity (inches per second): ", driveVel);
  frc::SmartDashboard::PutNumber(moduleName + "Drive position velocity (inches per second): ", drivePosVel);

  frc::SmartDashboard::PutNumber(moduleName + "Steer motor raw position: ", steerMotorPosition);
  frc::SmartDashboard::PutNumber(moduleName + "Steer encoder position: ", steerEncoderPosition);
  frc::SmartDashboard::PutNumber(moduleName + "Steer encoder absolute position: ", steerEncoderAbsolutePosition);
  // loopcount, drive position, drive motor velocity, steer motor position, steer motor velocity, steer encoder absolute position, steer encoder position, steer encoder velocity, can pct utilization
  // printf("%f, %f, %f, %f, %f, %f[], %f, %f, %f\n",mainRobot->GetLoopCount(), drivePosition, driveMotorVel,steerMotorPosition, steerMotorVel,steerEncoderAbsolutePosition, steerEncoderPosition,steerEncoderVel,(frc::RobotController::GetCANStatus().percentBusUtilization*100));
}
/**
 * @brief Collect sensor states
 * @note Negated sensor values according to correct motor direction
 *
 */
void SwerveModule::Analyze()
{
  drivePosition = -drive->GetSelectedSensorPosition();
  driveMotorVel = -drive->GetSelectedSensorVelocity();

  drivePosVel = (drivePosition - driveLastPosition) * GetPositionVelocityScaleFactor();
  driveVel = driveMotorVel * GetVelocityScaleFactor();
  driveLastPosition = drivePosition;

  steerMotorPosition = -steer->GetSelectedSensorPosition();
  steerMotorVel = -steer->GetSelectedSensorVelocity();

  // adjust abs sensor range from 0-360 to -180 to 180
  steerEncoderAbsolutePosition = -steerEncoder->GetAbsolutePosition() + 180;

  steerEncoderPosition = -steerEncoder->GetPosition();
  steerEncoderVel = -steerEncoder->GetVelocity();
}

/**
 * @brief Control swerve module
 *
 * @param vel velocity in inches per second
 * @param angle angle cmd from -180 to 180
 */
void SwerveModule::DriveControl(double vel, double angle)
{
  steer_pErr = angle - steerEncoderAbsolutePosition + 180 * rev;
  if (fabs(Robot::Wrap(steer_pErr)) > 90)
  {
    rev = !rev;
    steer_pErr += 180;
  }
  steer_pErr = Robot::Wrap(steer_pErr);
  steer_vCmd = STEER_kP * steer_pErr;
  steer_vErr = steer_vCmd - steerEncoderVel;
  steer_vErrIntegrator += steer_vErr * LOOPTIME;
  steer_vErrIntegrator = Robot::Limit(-STEER_kR / STEER_kV, STEER_kR / STEER_kV, steer_vErrIntegrator);
  steer_cmd = (STEER_kV / STEER_kR) * (steer_vErr * STEER_TAU + steer_vErrIntegrator);

  if (rev)
  {
    vel *= -1;
  }

  SetWheelCmd(steer_cmd);

  drive_vErr = vel - driveVel + kAV * steer_vCmd;
  drive_vErrIntegrator += drive_vErr * LOOPTIME;
  drive_vErrIntegrator = Robot::Limit(-DRIVE_kR / DRIVE_kV, DRIVE_kR / DRIVE_kV, drive_vErrIntegrator);
  drive_cmd = (DRIVE_kV / DRIVE_kR) * (drive_vErr * DRIVE_TAU + drive_vErrIntegrator);

  SetDriveCmd(drive_cmd);
}