// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/subsystem_Intake.h"

subsystem_Intake::subsystem_Intake() : m_IntakeMotor{IntakeConstants::IntakeMotorID, rev::CANSparkMaxLowLevel::MotorType::kBrushless},
                                       m_Solenoid{frc::PneumaticsModuleType::CTREPCM, IntakeConstants::IntakePistonA, IntakeConstants::IntakePistonB},
                                       m_ColorSensor{frc::I2C::Port::kOnboard},
                                       m_ColorMatcher{} {
    m_ColorMatcher.AddColorMatch(ColorConstants::PurpleTarget);
    m_ColorMatcher.AddColorMatch(ColorConstants::YellowTarget);
}

void subsystem_Intake::IntakeClose() {
    m_Solenoid.Set(frc::DoubleSolenoid::kReverse);
    m_IsOpen = false;
}

void subsystem_Intake::IntakeOpen() {
    m_Solenoid.Set(frc::DoubleSolenoid::kForward);
    m_IsOpen = true;
}

void subsystem_Intake::SetIntakeWheelsOn(bool IsForward) {
    if(IsForward) {
        m_IntakeMotor.Set(IntakeConstants::OutputPower);
    } else {
        m_IntakeMotor.Set(-IntakeConstants::OutputPower);
    }
}

void subsystem_Intake::SetIntakeWheelsOff() {
    m_IntakeMotor.Set(0.0);
}

bool subsystem_Intake::IsIntakeOpen() {
    return m_IsOpen;
}

void subsystem_Intake::OuttakeCube() {
    SetIntakeWheelsOn(false);
    IntakeOpen();
}

std::string subsystem_Intake::GetCurrentState() {
    return m_CurrentState;
}

uint32_t subsystem_Intake::GetCurrentProximity() {
    return m_CurrentProximity;
}

// This method will be called once per scheduler run
void subsystem_Intake::Periodic() {
    
    // Current Proximity (changes member variable curr proximity)
    m_CurrentProximity = m_ColorSensor.GetProximity();

    // Current Color (changes member variable curr color)
    m_CurrentColor = m_ColorSensor.GetColor();

    // MatchedColor (gives you the detected color)
    double Confidence = 0.0;
    frc::Color MatchedColor = m_ColorMatcher.MatchClosestColor(m_CurrentColor, Confidence);

    // Checking Instantaneous State (sets instant state to whatever it detects at the moment)
    std::string InstStateStr;

    if(m_CurrentProximity >= ColorConstants::ProximityTarget) {
        if(MatchedColor == ColorConstants::PurpleTarget) {
            InstStateStr = "Purple";
        } else if(MatchedColor == ColorConstants::YellowTarget) {
            InstStateStr = "Yellow";
        }
    } else {
        InstStateStr = "Empty";
    }

    // Experiment Area/Field
    if(m_PreviousColor != m_CurrentColor) {
        m_ColorChangeCount = 0;
    }
    
    if(m_ColorChangeCount >= 3) {
        m_CurrentState = InstStateStr;
    }

    m_PreviousColor = m_CurrentColor;
    m_ColorChangeCount++;

    // Display Color Sensor info to SmartDashboard
    frc::SmartDashboard::PutNumber("Confidence", Confidence);
    frc::SmartDashboard::PutNumber("Proximity", m_CurrentProximity);
    frc::SmartDashboard::PutNumber("Red", m_CurrentColor.red);
    frc::SmartDashboard::PutNumber("Green", m_CurrentColor.green);
    frc::SmartDashboard::PutNumber("Blue", m_CurrentColor.blue);
    frc::SmartDashboard::PutString("Current State", m_CurrentState);
}
