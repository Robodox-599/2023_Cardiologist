// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/subsystem_Intake.h"

subsystem_Intake::subsystem_Intake() : m_IntakeMotor{IntakeConstants::IntakeMotorID, rev::CANSparkMaxLowLevel::MotorType::kBrushless},
                                       m_IntakeMotorPID{m_IntakeMotor.GetPIDController()},
                                       m_IntakeEncoder{m_IntakeMotor.GetEncoder()},
                                       m_Solenoid{frc::PneumaticsModuleType::CTREPCM, IntakeConstants::IntakePistonA, IntakeConstants::IntakePistonB},
                                       m_ColorSensor{frc::I2C::Port::kOnboard},
                                       m_ColorMatcher{},
                                       m_ProximityPID{IntakeConstants::kProximityP, 0.0, IntakeConstants::kProximityD}
{
    // m_IntakeMotor.SetSmartCurrentLimit(IntakeConstants::CurrentLimit);
    // m_IntakeMotorPID.SetSmartMotionMaxVelocity(IntakeConstants::MaxVelocity);
    m_IntakeMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    // m_IntakeMotorPID.SetP(IntakeConstants::kIntakeP);
    // m_IntakeMotorPID.SetI(IntakeConstants::kIntakeI);
    // m_IntakeMotorPID.SetD(IntakeConstants::kIntakeD);
    // m_IntakeMotorPID.SetFF(IntakeConstants::kIntakeFF);
    m_ColorMatcher.AddColorMatch(ColorConstants::PurpleTarget);
    m_ColorMatcher.AddColorMatch(ColorConstants::YellowTarget);
    m_ProximityPID.SetSetpoint(ColorConstants::TargetProximity);
}

void subsystem_Intake::IntakeClose()
{
    m_Solenoid.Set(frc::DoubleSolenoid::kReverse);
    m_IsOpen = false;
}

void subsystem_Intake::IntakeOpen()
{
    m_Solenoid.Set(frc::DoubleSolenoid::kForward);
    m_IsOpen = true;
}

bool subsystem_Intake::IsIntakeOpen()
{
    return m_IsOpen;
}

void subsystem_Intake::SetIntakeWheelsOutput(bool IsIntaking)
{
    if (IsIntaking)
    {
        m_DesiredOutput = m_ProximityPID.Calculate(m_CurrentProximity);
        // m_DesiredOutput = IntakeConstants::IntakePower / m_CurrentProximity * IntakeConstants::ProxToVelocity;
        // Power / Current Proximity (so that speed of wheels decrease as object is closer)
        m_IntakeMotor.Set(m_DesiredOutput);
        // m_IntakeMotorPID.SetReference(-m_DesiredOutput, rev::CANSparkMaxLowLevel::ControlType::kVelocity);
    }
    else
    {
        m_DesiredOutput = IntakeConstants::OuttakePower;
        m_IntakeMotor.Set(m_DesiredOutput);
        // m_IntakeMotorPID.SetReference(m_DesiredOutput, rev::CANSparkMaxLowLevel::ControlType::kVelocity);
    }
}

IntakeConstants::State subsystem_Intake::GetCurrentState(){
    return m_CurrentState;
}

void subsystem_Intake::SetIntakeWheelsOff()
{
    m_IntakeMotor.Set(0.0);
}


double subsystem_Intake::GetCurrentProximity()
{
    return m_CurrentProximity;
}

void subsystem_Intake::SetVelocity(double velocity)
{
    m_IntakeMotorPID.SetReference(velocity, rev::ControlType::kVelocity);
}

void subsystem_Intake::SetHighCubeDynamicVelocity(units::meter_t YBotPosition)
{
    units::meter_t DistanceFromTarget = YBotPosition - IntakeConstants::CenterToStowedIntake - IntakeConstants::DistanceToHighCube;

    
    
}

void subsystem_Intake::SetMidCubeDynamicVelocity(units::meter_t YBotPosition)
{
    units::meter_t DistanceFromTarget = YBotPosition - IntakeConstants::CenterToStowedIntake - IntakeConstants::DistanceToMidCube;

    
}



// This method will be called once per scheduler run
void subsystem_Intake::Periodic()
{



    frc::SmartDashboard::PutBoolean("Solenoid FWD", m_Solenoid.IsFwdSolenoidDisabled());

    frc::SmartDashboard::PutBoolean("Solenoid REV", m_Solenoid.IsRevSolenoidDisabled());
    frc::SmartDashboard::PutNumber("Solenoid Direction", m_Solenoid.Get());

    // Current Proximity (changes member variable curr proximity)
    // 10000.0 / 1.2 is the constant that transforms proximity into a number out of 100
    m_CurrentProximity = (10000.0 / 1.2) / m_ColorSensor.GetProximity();

    // Current Color (changes member variable curr color)
    m_CurrentColor = m_ColorSensor.GetColor();

    // MatchedColor (gives you the detected color)
    double Confidence = 0.0;
    frc::Color MatchedColor = m_ColorMatcher.MatchClosestColor(m_CurrentColor, Confidence);

    // Checking Instantaneous State (sets instant state to whatever it detects at the moment)
    IntakeConstants::State InstStateStr = IntakeConstants::State::Nothing;

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
        InstStateStr = IntakeConstants::State::Nothing;
    }

    // Enum stuff
    if (m_PreviousColor != m_CurrentColor)
    {
        m_ColorChangeCount = 0;
    }

    if (m_ColorChangeCount >= 3)
    {
        m_CurrentState = InstStateStr;
    }

    m_PreviousColor = m_CurrentColor;
    m_ColorChangeCount++;

    frc::SmartDashboard::PutNumber("Intake P", 0);
    frc::SmartDashboard::PutNumber("Intake I", 0);
    frc::SmartDashboard::PutNumber("Intake D", 0);
    frc::SmartDashboard::PutNumber("Intake FF", 0);
    frc::SmartDashboard::PutNumber("Desired Intake Vel", 0);

    m_IntakeMotorPID.SetP(frc::SmartDashboard::GetNumber("Intake P", 0));
    m_IntakeMotorPID.SetI(frc::SmartDashboard::GetNumber("Intake I", 0));
    m_IntakeMotorPID.SetD(frc::SmartDashboard::GetNumber("Intake D", 0));
    m_IntakeMotorPID.SetFF(frc::SmartDashboard::GetNumber("Intake FF", 0));
    m_IntakeMotorPID.SetReference(frc::SmartDashboard::GetNumber("Desired Intake Vel", 0), rev::ControlType::kVelocity);
    
    // // Display encoder info
    // frc::SmartDashboard::PutNumber("Desired Output", m_DesiredOutput);
    // frc::SmartDashboard::PutNumber("Actual Velocity", m_IntakeEncoder.GetVelocity());

    // // Display Color Sensor info to SmartDashboard
    // frc::SmartDashboard::PutNumber("Confidence", Confidence);
    // frc::SmartDashboard::PutNumber("Proximity", m_CurrentProximity);
    // frc::SmartDashboard::PutNumber("Red", m_CurrentColor.red);
    // frc::SmartDashboard::PutNumber("Green", m_CurrentColor.green);
    // frc::SmartDashboard::PutNumber("Blue", m_CurrentColor.blue);
    // frc::SmartDashboard::PutNumber("Current State", m_CurrentState);
}
