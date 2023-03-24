// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/subsystem_Intake.h"

subsystem_Intake::subsystem_Intake() : /*m_IntakeMotor{IntakeConstants::IntakeMotorID, rev::CANSparkMaxLowLevel::MotorType::kBrushless},
                                       m_IntakeMotorPID{m_IntakeMotor.GetPIDController()},
                                       m_IntakeEncoder{m_IntakeMotor.GetEncoder()},*/
                                       m_Solenoid{frc::PneumaticsModuleType::CTREPCM, IntakeConstants::IntakePistonA, IntakeConstants::IntakePistonB},
                                       m_ColorSensor{frc::I2C::Port::kOnboard},
                                       m_ColorMatcher{},
                                       m_ProximityPID{IntakeConstants::kProximityP, 0.0, IntakeConstants::kProximityD}
{
    // m_IntakeMotor.SetSmartCurrentLimit(25);
    // // m_IntakeMotorPID.SetSmartMotionMaxVelocity(IntakeConstants::MaxVelocity);
    // m_IntakeMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
    // m_IntakeMotorPID.SetP(0.01);
    // m_IntakeMotorPID.SetI(0.0);
    // m_IntakeMotorPID.SetD(0.0);
    m_ColorMatcher.AddColorMatch(ColorConstants::PurpleTarget);
    m_ColorMatcher.AddColorMatch(ColorConstants::YellowTarget);
    m_ProximityPID.SetSetpoint(ColorConstants::TargetProximity);
    m_Solenoid.Set(frc::DoubleSolenoid::kForward);
    
}

void subsystem_Intake::IntakeClose()
{
    m_Solenoid.Set(frc::DoubleSolenoid::kForward);
    m_IsOpen = false;
}

void subsystem_Intake::IntakeOpen()
{
    m_Solenoid.Set(frc::DoubleSolenoid::kReverse);
    m_IsOpen = true;
}

bool subsystem_Intake::IsIntakeOpen()
{
    return m_IsOpen;
}

// void subsystem_Intake::SetIntakeWheelsOutput(bool IsIntaking)
// {
//     if (IsIntaking)
//     {
//         m_DesiredOutput = m_ProximityPID.Calculate(m_CurrentProximity);
//         // m_DesiredOutput = IntakeConstants::IntakePower / m_CurrentProximity * IntakeConstants::ProxToVelocity;
//         // Power / Current Proximity (so that speed of wheels decrease as object is closer)
//         // m_IntakeMotor.Set(m_DesiredOutput);
//         // m_IntakeMotorPID.SetReference(-m_DesiredOutput, rev::CANSparkMaxLowLevel::ControlType::kVelocity);
//     }
//     else
//     {
//         m_DesiredOutput = IntakeConstants::OuttakePower;
//         // m_IntakeMotor.Set(m_DesiredOutput);
//         // m_IntakeMotorPID.SetReference(m_DesiredOutput, rev::CANSparkMaxLowLevel::ControlType::kVelocity);
//     }
// }

IntakeConstants::State subsystem_Intake::GetCurrentState(){
    return m_CurrentState;
}



double subsystem_Intake::GetCurrentProximity()
{
    return m_CurrentProximity;
}



void subsystem_Intake::SetOff(){
    m_CurrentMode = IntakeConstants::IntakeMode::Off;
}

void subsystem_Intake::SetPassive(){
    m_CurrentMode = IntakeConstants::IntakeMode::Passive;
}

void subsystem_Intake::SetIntake(){
    m_CurrentMode = IntakeConstants::IntakeMode::Intake;
}

void subsystem_Intake::SetOutake(){
    m_CurrentMode = IntakeConstants::IntakeMode::Outake;
}

void subsystem_Intake::SetHighCubeStaticVelocity()
{
    m_CurrentMode = IntakeConstants::IntakeMode::HighShoot;
}

void subsystem_Intake::SetMidCubeStaticVelocity()
{
    m_CurrentMode = IntakeConstants::IntakeMode::MidShoot;
}


void subsystem_Intake::MaintainIntakeMode(){
    // switch(m_CurrentMode){
    //     case(IntakeConstants::IntakeMode::Off):
    //         m_IntakeMotor.Set(0.0);
    //         break;
    //     case(IntakeConstants::IntakeMode::Passive):
    //         m_IntakeMotor.Set(IntakeConstants::PassivePower);
    //         break;
    //     case(IntakeConstants::IntakeMode::Intake):
    //         m_IntakeMotor.Set(m_ProximityPID.Calculate(m_CurrentProximity));
    //         break;
    //     case(IntakeConstants::IntakeMode::Outake):
    //         m_IntakeMotor.Set(IntakeConstants::OuttakePower);
    //         break;
    //     case(IntakeConstants::IntakeMode::MidShoot):
    //         m_IntakeMotorPID.SetReference(4000, rev::CANSparkMaxLowLevel::ControlType::kVelocity, 0, 5);
    //         break;
    //     case(IntakeConstants::IntakeMode::HighShoot):
    //         m_IntakeMotorPID.SetReference(6000, rev::CANSparkMaxLowLevel::ControlType::kVelocity, 0, 7);
    //         break;
    // }
}

frc::Color subsystem_Intake::GetColor(){
    return m_CurrentColor;
}



// This method will be called once per scheduler run
void subsystem_Intake::Periodic()
{



    // m_IntakeMotorPID.SetReference(7000, rev::ControlType::kVelocity, 0, 10);

    // frc::SmartDashboard::PutNumber("Velocity", m_IntakeEncoder.GetVelocity());
    // frc::SmartDashboard::PutNumber("Graph Velocity", m_IntakeEncoder.GetVelocity());

    MaintainIntakeMode();

    // Current Proximity (changes member variable curr proximity)
    // 10000.0 / 1.2 is the constant that transforms proximity into a number out of 100
    m_CurrentProximity = (10000.0 / 1.2) / m_ColorSensor.GetProximity();

    // Current Color (changes member variable curr color)
    m_CurrentColor = m_ColorSensor.GetColor();

    // MatchedColor (gives you the detected color)
    double Confidence = 0.0;
    frc::Color MatchedColor = m_ColorMatcher.MatchClosestColor(m_CurrentColor, Confidence);

    // Checking Instantaneous State (sets instant state to whatever it detects at the moment)
    IntakeConstants::State InstStateStr = IntakeConstants::State::Empty;

    if (m_CurrentProximity <= ColorConstants::RecognitionProximity)
    {
        if (MatchedColor == ColorConstants::PurpleTarget)
        {
            InstStateStr = IntakeConstants::State::Purple;
        }
        else if (MatchedColor == ColorConstants::YellowTarget)
        {
            InstStateStr = IntakeConstants::State::Yellow;
        }
    }
    else
    {
        InstStateStr = IntakeConstants::State::Empty;
    }

    // Enum stuff
    if (m_PreviousColor != m_CurrentColor)
    {
        m_ColorChangeCount = 0;
    }

    if (m_ColorChangeCount >= 2)
    {
        m_CurrentState = InstStateStr;
    }

    m_PreviousColor = m_CurrentColor;
    m_ColorChangeCount++;

    // frc::SmartDashboard::PutNumber("Intake P", 0);
    // frc::SmartDashboard::PutNumber("Intake I", 0);
    // frc::SmartDashboard::PutNumber("Intake D", 0);
    // frc::SmartDashboard::PutNumber("Intake FF", 0);
    // frc::SmartDashboard::PutNumber("Desired Intake Vel", 0);

    // m_IntakeMotorPID.SetP(frc::SmartDashboard::GetNumber("Intake P", 0));
    // m_IntakeMotorPID.SetI(frc::SmartDashboard::GetNumber("Intake I", 0));
    // m_IntakeMotorPID.SetD(frc::SmartDashboard::GetNumber("Intake D", 0));
    // m_IntakeMotorPID.SetFF(frc::SmartDashboard::GetNumber("Intake FF", 0));
    // m_IntakeMotorPID.SetReference(frc::SmartDashboard::GetNumber("Desired Intake Vel", 0), rev::ControlType::kVelocity);
    
    // // Display encoder info
    // frc::SmartDashboard::PutNumber("Desired Output", m_DesiredOutput);
    // frc::SmartDashboard::PutNumber("Actual Velocity", m_IntakeEncoder.GetVelocity());


    // Display IntakeMode to SmartDashboard
    switch(m_CurrentMode){
        case(IntakeConstants::IntakeMode::Off):
            frc::SmartDashboard::PutString("IntakeMode", "OFF");
            break;
        case(IntakeConstants::IntakeMode::Passive):
            frc::SmartDashboard::PutString("IntakeMode", "Passive");
            break;
        case(IntakeConstants::IntakeMode::Intake):
            frc::SmartDashboard::PutString("IntakeMode", "Intake");
            break;
        case(IntakeConstants::IntakeMode::Outake):
            frc::SmartDashboard::PutString("IntakeMode", "Outake");
            break;
        case(IntakeConstants::IntakeMode::MidShoot):
            frc::SmartDashboard::PutString("IntakeMode", "MidShoot");
            break;
        case(IntakeConstants::IntakeMode::HighShoot):
            frc::SmartDashboard::PutString("IntakeMode", "HighShoot");
            break;
    }

    // // Display Color Sensor info to SmartDashboard
    frc::SmartDashboard::PutNumber("Proximity", m_CurrentProximity);
    if(m_CurrentState == IntakeConstants::State::Empty){
        frc::SmartDashboard::PutString("Intake State", "Empty");
    }else if(m_CurrentState == IntakeConstants::State::Yellow){
        frc::SmartDashboard::PutString("Intake State", "Cone");
    }else if(m_CurrentState == IntakeConstants::State::Purple){
        frc::SmartDashboard::PutString("Intake State", "Cube");
    }

}
