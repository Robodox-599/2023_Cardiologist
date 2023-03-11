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
#include <frc/PowerDistribution.h>
#include <units/length.h>
#include <units/angle.h>
#include "Constants.h"

class subsystem_Arm : public frc2::SubsystemBase
{
public:
  subsystem_Arm();

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

  void ResetWrist();

  

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
  
  frc::DoubleSolenoid m_ElbowBrake;
  frc::DoubleSolenoid m_ShoulderBrake;






};
