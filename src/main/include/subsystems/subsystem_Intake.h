// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include "Constants.h"
#include "rev/CANSparkMax.h"
#include "frc/DoubleSolenoid.h"

class subsystem_Intake : public frc2::SubsystemBase {
 public:
  subsystem_Intake();
  
  void IntakeClose();
  void IntakeOpen();
  void SetIntakeWheelsOn(double speed);
  bool IsIntakeOpen();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
  rev::CANSparkMax m_IntakeMotor;
  frc::DoubleSolenoid m_LeftSolenoid;
  frc::DoubleSolenoid m_RightSolenoid;

  bool m_IsClamped = true;
};
