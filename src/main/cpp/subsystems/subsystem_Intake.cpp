// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/subsystem_Intake.h"

subsystem_Intake::subsystem_Intake() : m_IntakeMotor{IntakeConstants::IntakeMotorID, rev::CANSparkMaxLowLevel::MotorType::kBrushless} {
    
}

void subsystem_Intake::SetIntakeWheelsOn(double outputPower) {
    m_IntakeMotor.Set(outputPower);
}

// This method will be called once per scheduler run
void subsystem_Intake::Periodic() {}
