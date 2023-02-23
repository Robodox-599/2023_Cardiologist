// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <rev/CANSparkMax.h>
#include "frc/DoubleSolenoid.h"
#include <frc/DigitalInput.h>
#include <frc/Timer.h>
#include "Constants.h"


class subsystem_Arm : public frc2::SubsystemBase {
 public:
  subsystem_Arm();

  double CalculateBottomArmAngle(double x, double y);
  double CalculateTopArmAngle(double x, double y);

  double EncoderToDegrees(double ticks);
  double DegreesToSmartMotion(double degrees);

  void MoveArm(double x, double y);
  void MoveArmManually(double leftAxis, double rightAxis);
  void SetIntakeAngle(double angle);

  void LockArm();
  void UnlockArm();

  void SetToZero();

  void SetArmPIDByDirection(double x, double y);
  void SetArmPIDByDirection(bool leftJoy, bool rightJoy);
  bool CheckHallEffect();

  // True is positive direction ; False is negative direction (Jackson change this later bc its probably wrong)
  bool IsTopArmDirectionGoingUp(double x, double y);
  bool IsBottomArmDirectionGoingUp(double x, double y);

  bool IsAtDesiredPosition();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

 private:
  double armX = 0.0;
  double armY = 0.0;
  double adjustedX = 0.0;
  double adjustedY = 0.0;
  double manualX = 0.0;
  double manualY = 0.0;

  double topPosition;
  double bottomPosition;

  double convertedTop;
  double convertedBottom;

  double bottomStartPos;
  double topStartPos; 

  double intakeAngleOffset; 

  double bottomAngle;
  double topAngle; 

  rev::CANSparkMax m_BottomArmMotor;
  rev::CANSparkMax m_BottomFollower; 
  rev::CANSparkMax m_TopArmMotor;
  rev::CANSparkMax m_TopFollower; 
  rev::CANSparkMax m_IntakeTiltMotor; 

  rev::SparkMaxPIDController m_BottomArmPID;
  rev::SparkMaxPIDController m_TopArmPID;
  rev::SparkMaxPIDController m_IntakeTiltPID; 
  
  frc::DoubleSolenoid m_TopSolenoid;
  frc::DoubleSolenoid m_BottomSolenoid;

  rev::SparkMaxAbsoluteEncoder m_TopAbsEncoder;
  rev::SparkMaxAbsoluteEncoder m_BottomAbsEncoder;

  rev::SparkMaxRelativeEncoder m_BottomRelEncoder;
  rev::SparkMaxRelativeEncoder m_TopRelEncoder; 
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
};
