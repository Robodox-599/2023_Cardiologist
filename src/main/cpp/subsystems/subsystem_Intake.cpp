// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/subsystem_Intake.h"

subsystem_Intake::subsystem_Intake() : m_IntakeMotor{IntakeConstants::IntakeMotorID, rev::CANSparkMaxLowLevel::MotorType::kBrushless},
                                       m_IntakeMotorPID{m_IntakeMotor.GetPIDController()},
                                       m_IntakeEncoder{m_IntakeMotor.GetEncoder()},
                                       m_Solenoid{frc::PneumaticsModuleType::CTREPCM, IntakeConstants::IntakePistonA, IntakeConstants::IntakePistonB}
{
    //m_IntakeMotor.SetSmartCurrentLimit(IntakeConstants::CurrentLimit);
    //m_IntakeMotorPID.SetSmartMotionMaxVelocity(IntakeConstants::MaxVelocity);
    m_IntakeMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    m_IntakeMotorPID.SetP(IntakeConstants::kIntakeP);
    m_IntakeMotorPID.SetD(IntakeConstants::kIntakeD);
    m_IntakeMotorPID.SetFF(IntakeConstants::kIntakeFF);

}

void subsystem_Intake::IntakeClose() {
    m_Solenoid.Set(frc::DoubleSolenoid::kReverse);
    m_IsOpen = false;
}

void subsystem_Intake::IntakeOpen() {
    m_Solenoid.Set(frc::DoubleSolenoid::kForward);
    m_IsOpen = true;
}

bool subsystem_Intake::IsIntakeOpen() {
    return m_IsOpen;
}

void subsystem_Intake::SetIntake(){
    m_State = IntakeConstants::IntakeMode::Intake;
}

void subsystem_Intake::SetOutake(){
    m_State = IntakeConstants::IntakeMode::Outake;
}

void subsystem_Intake::SetPassive(){
    m_State = IntakeConstants::IntakeMode::Passive;
}

void subsystem_Intake::SetOff(){
    m_State = IntakeConstants::IntakeMode::Off;
}

IntakeConstants::IntakeMode subsystem_Intake::GetState(){
    return m_State;
}

double subsystem_Intake::GetCurrent(){
    return m_IntakeMotor.GetOutputCurrent();
    
}



void subsystem_Intake::SetState(){
    switch(m_State){
        case IntakeConstants::IntakeMode::Passive:
            m_IntakeMotor.Set(IntakeConstants::PassivePower);
            break;
        case IntakeConstants::IntakeMode::Intake:
            m_IntakeMotor.Set(IntakeConstants::IntakePower);
            break;
        case IntakeConstants::IntakeMode::Outake:
            m_IntakeMotor.Set(IntakeConstants::OuttakePower);
            break;
        case IntakeConstants::IntakeMode::Off:
            m_IntakeMotor.Set(0.0);
            break;
    }

}




// This method will be called once per scheduler run
void subsystem_Intake::Periodic() {
    SetState();




    // Current Proximity (changes member variable curr proximity)
    // 10000.0 / 1.2 is the constant that transforms proximity into a number out of 100

    // Current Color (changes member variable curr color)

    // MatchedColor (gives you the detected color)

    // Checking Instantaneous State (sets instant state to whatever it detects at the moment)



    // Enum stuff



}
