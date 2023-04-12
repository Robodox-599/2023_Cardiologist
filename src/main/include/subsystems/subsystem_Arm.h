// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <rev/CANSparkMax.h>
#include <frc2/command/FunctionalCommand.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/WaitCommand.h>
#include <frc2/command/Commands.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc/DutyCycleEncoder.h>
#include <frc/controller/ArmFeedforward.h>
#include "frc/DoubleSolenoid.h"
#include <frc/DigitalInput.h>
#include <frc/Timer.h>
#include <frc/PowerDistribution.h>
#include <units/length.h>
#include <units/angle.h>
#include "Constants.h"

class subsystem_Arm : public frc2::SubsystemBase
{
public:
  subsystem_Arm();

  void PollArmPosition(int POV);
  DPAD::NODE_LEVEL GetArmPoll();


  //Might use these methods later. For now, we are relying on motor rotations instead of coordinates.
  double CalculateShoulderAngle(double x, double y);
  double CalculateElbowAngle(double x, double y);

  double EncoderToDegrees(double ticks);

  void MoveArm(double x, double y);
  void MoveArmManually(double leftAxis, double rightAxis);
  void TiltWristManually(double trigger);

  //Methods for brakes on the arm. Currently deprecated because we don't even need brakes to hold the arm up. 
  void LockArm();
  void UnlockArm();

  void RunArmTest(double Shoulder, double Elbow, double Wrist);
  void RunArmManualTest(double leftStick, double rightStick);
  void SetShoulderByPosition(double ShoulderPos);
  void SetElbowByPosition(double ElbowPos);
  void SetWristByPosition(double WristPos);
  
  double GetShoulderIncrement();
  double GetWristIncrement();
  double GetElbowIncrement();

  bool ElbowThreshold(double Threshold);
  bool WristThreshold(double Threshold);
  bool ShoulderThreshold(double Threshold);

  bool IsCubeMode();
  void ChangeGamePieceMode();
  frc2::CommandPtr ResetWrist();
  frc2::CommandPtr ToHighCone();
  frc2::CommandPtr ToMidCone();
  frc2::CommandPtr ToHighCube();
  frc2::CommandPtr ToMidCube();

  frc2::CommandPtr ConeMovement();
  frc2::CommandPtr CubeMovement();

  units::angle::radian_t RotationsToRadians(double rotations);

  void WristCommandStart(double WristPos);
  void WristCommandExecute();
  bool WristCommandIsFinished(bool IsWait, double Threshold);

  void ElbowCommandStart(double ElbowPos);
  void ElbowCommandExecute();
  bool ElbowCommandIsFinished(bool IsWait, double Threshold);

  void ShoulderCommandStart(double ShoulderPos);
  void ShoulderCommandExecute();
  bool ShoulderCommandIsFinished(bool IsWait);

  void SetElbowPIDByDirection(double desiredElbowPos);
  void SetShoulderPIDByDirection(double desiredShoulderPos);

  double GetElbowPosition();
  double GetShoulderPosition();
  double GetWristPosition();


  // True is upward direction ; False is downward direction
  bool IsElbowDirectionGoingUp(double Elbow);
  bool IsShoulderDirectionGoingUp(double shoulder);

  bool IsElbowAtDesiredPosition();
  bool IsShoulderAtDesiredPosition(); 
  bool IsWristAtDesiredPosition();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

private:
  DPAD::NODE_LEVEL m_ArmPoll = DPAD::NODE_LEVEL::NON_SPECIFIED; 

  double power = 0.0;
  double armX = 0.0;
  double armY = 0.0;
  double adjustedX = 0.0;
  double adjustedY = 0.0;
  double manualX = 0.0;
  double manualY = 0.0;

  double ElbowPosition = 3.5;
  double ShoulderPosition = 0.0;
  double WristPosition = -20.925; 

  double ElbowEnc = 0.0;
  double ShoulderEnc = 0.0;
  double WristEnc = 0.0;

  double DesiredElbowPosition = 3.5;
  double DesiredShoulderPosition = 0.0;
  double DesiredWristPostion = -20.925;

  units::angle::radian_t DesiredShoulderRadians;
  units::angle::radian_t DesiredElbowRadians;  
  units::angle::radian_t DesiredWristRadians;

  double convertedElbow;
  double convertedShoulder;

  double Power4Shoulder;
  double Power4Elbow;

  double ShoulderStartPos;
  double ElbowStartPos;

  double ShoulderAngle;
  double ElbowAngle;
  double WristAngle;

  double ShoulderJointTheta;
  double ElbowJointTheta;
  double WristTheta;

  int m_ElbowSlot = 0;
  int m_ShoulderSlot = 0;

  bool m_IsManual = false;
  bool m_IsCubeMode = false;

  rev::CANSparkMax m_ShoulderMotor;
  rev::CANSparkMax m_ShoulderFollower;
  rev::CANSparkMax m_ElbowMotor;
  rev::CANSparkMax m_ElbowFollower;
  rev::CANSparkMax m_WristMotor;

  rev::SparkMaxPIDController m_ShoulderPID;
  rev::SparkMaxPIDController m_ShoulderFollowerPID;
  rev::SparkMaxPIDController m_ElbowPID;
  rev::SparkMaxPIDController m_ElbowFollowerPID;
  rev::SparkMaxPIDController m_WristPID;

  rev::SparkMaxRelativeEncoder m_ShoulderRelEncoder;
  rev::SparkMaxRelativeEncoder m_ElbowRelEncoder;
  rev::SparkMaxRelativeEncoder m_WristEncoder; 

  rev::SparkMaxRelativeEncoder m_ShoulderRelFollowerEncoder;
  rev::SparkMaxRelativeEncoder m_ElbowRelFollowerEncoder;

  rev::SparkMaxLimitSwitch m_BackLimit;
  rev::SparkMaxLimitSwitch m_FrontLimit;

  frc::PowerDistribution m_PDH;

  frc::DutyCycleEncoder m_ElbowAbsEncoder;
  frc::DutyCycleEncoder m_WristAbsEncoder;

  frc::Timer m_WristTimer{};
  frc::Timer m_ElbowTimer{};
  frc::Timer m_ShoulderTimer{};

  frc::ArmFeedforward m_ElbowFeedforward;
  units::volt_t ElbowFF;
  frc::ArmFeedforward m_ShoulderFeedforward;
  units::volt_t ShoulderFF; 
  frc::ArmFeedforward m_WristFeedforward;
  units::volt_t WristFF;
 double Increment = 0.0;

};
