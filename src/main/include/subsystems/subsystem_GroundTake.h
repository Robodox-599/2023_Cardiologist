// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc/AnalogInput.h>
#include <rev/CANSparkMax.h>
#include "Constants.h"
#include <frc2/command/CommandPtr.h>



class subsystem_GroundTake : public frc2::SubsystemBase {
 public:
  subsystem_GroundTake();
  bool IsCubeDetected();
  void ExtendGroundTake();
  void RetractGroundTake();
  frc2::CommandPtr RunIntakeToggleCommand();
  frc2::CommandPtr RunIntakeCommand();
  frc2::CommandPtr ExtendGroundTakeCommand();
  frc2::CommandPtr RetractGroundTakeCommand();
  frc2::CommandPtr StopIntakeCommand();
  void RunIntake();
  void StopIntake();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
  frc::AnalogInput m_LeftBeamBreak;
  frc::AnalogInput m_CenterBeamBreak;
  frc::AnalogInput m_RightBeamBreak;

  rev::CANSparkMax m_ExtenderMotor;
  rev::CANSparkMax m_IntakeMotor;

  rev::SparkMaxPIDController m_ExtenderPID;
  rev::SparkMaxPIDController m_IntakePID;

  rev::SparkMaxRelativeEncoder m_ExtenderRelEncoder;
  rev::SparkMaxRelativeEncoder m_IntakeRelEncoder;

  
};
