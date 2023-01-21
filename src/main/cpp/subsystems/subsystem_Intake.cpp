// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/subsystem_Intake.h"

subsystem_Intake::subsystem_Intake() : m_IntakeMotor{IntakeConstants::IntakeMotorID, rev::CANSparkMaxLowLevel::MotorType::kBrushless},
                                       m_LeftSolenoid{frc::PneumaticsModuleType::CTREPCM, IntakeConstants::IntakePistonLA, IntakeConstants::IntakePistonLB},
                                       m_RightSolenoid{frc::PneumaticsModuleType::CTREPCM, IntakeConstants::IntakePistonRA, IntakeConstants::IntakePistonRB} {

}

void subsystem_Intake::IntakeClose() {
    m_LeftSolenoid.Set(frc::DoubleSolenoid::kReverse);
    m_RightSolenoid.Set(frc::DoubleSolenoid::kReverse);
    m_IsClamped = false;
}

void subsystem_Intake::IntakeOpen() {
    m_LeftSolenoid.Set(frc::DoubleSolenoid::kForward);
    m_RightSolenoid.Set(frc::DoubleSolenoid::kForward);
    m_IsClamped = true;
}

void subsystem_Intake::SetIntakeWheelsOn(double outputPower) {
    m_IntakeMotor.Set(outputPower);
}

bool subsystem_Intake::IsIntakeOpen() {
    return m_IsClamped;
}

// This method will be called once per scheduler run
void subsystem_Intake::Periodic() {}
