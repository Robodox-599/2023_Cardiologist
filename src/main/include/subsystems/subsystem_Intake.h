// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include "Constants.h"

#include "rev/CANSparkMax.h"
#include "frc/DoubleSolenoid.h"
#include <frc/util/Color.h>
#include <rev/ColorSensorV3.h>
#include <rev/ColorMatch.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include <frc/controller/PIDController.h>

class subsystem_Intake : public frc2::SubsystemBase {
 public:
  subsystem_Intake();
  
  void IntakeClose();
  void IntakeOpen();
  bool IsIntakeOpen();
  void SetIntakeWheelsOn(bool IsIntakeDirection);
  void SetIntakeWheelsOff();

  std::string GetCurrentState();
  uint32_t GetCurrentProximity();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
  
  // Wheels for intaking objects
  rev::CANSparkMax m_IntakeMotor;
  rev::SparkMaxPIDController m_IntakeMotorPID;
  rev::SparkMaxRelativeEncoder m_IntakeEncoder;
  // Pistons for clamping
  frc::DoubleSolenoid m_Solenoid;

  bool m_IsOpen = true;

  double m_DesiredVelocity = 0.0;

  // Color Sensor stuff
  rev::ColorSensorV3 m_ColorSensor;
  rev::ColorMatch m_ColorMatcher;

  frc::Color m_CurrentColor = frc::Color(0.0, 0.0, 0.0);
  frc::Color m_PreviousColor = frc::Color(0.0, 0.0, 0.0);
  int m_ColorChangeCount = 0;
  uint32_t m_CurrentProximity = 0;
  std::string m_CurrentState = "Nothing";

  frc::PIDController m_ProximityPID;
};