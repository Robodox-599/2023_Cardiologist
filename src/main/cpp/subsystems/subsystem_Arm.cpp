// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/subsystem_Arm.h"

subsystem_Arm::subsystem_Arm() : m_ShoulderMotor{ArmConstants::ShoulderMotorID, rev::CANSparkMax::MotorType::kBrushless},
                                 m_ShoulderFollower{ArmConstants::ShoulderFollowerID, rev::CANSparkMax::MotorType::kBrushless},
                                 m_ElbowMotor{ArmConstants::ElbowMotorID, rev::CANSparkMax::MotorType::kBrushless},
                                 m_ElbowFollower{ArmConstants::ElbowFollowerID, rev::CANSparkMax::MotorType::kBrushless},
                                 m_WristMotor{ArmConstants::WristMotorID, rev::CANSparkMax::MotorType::kBrushless},
                                 m_ShoulderPID{m_ShoulderMotor.GetPIDController()},
                                 m_ShoulderFollowerPID{m_ShoulderFollower.GetPIDController()},
                                 m_ElbowPID{m_ElbowMotor.GetPIDController()},
                                 m_ElbowFollowerPID{m_ElbowFollower.GetPIDController()},
                                 m_WristPID{m_WristMotor.GetPIDController()},
                                 m_ShoulderRelEncoder{m_ShoulderMotor.GetEncoder()},
                                 m_ElbowRelEncoder{m_ElbowMotor.GetEncoder()},
                                 m_WristEncoder{m_WristMotor.GetEncoder()},
                                 m_ShoulderRelFollowerEncoder{m_ShoulderFollower.GetEncoder()},
                                 m_ElbowRelFollowerEncoder{m_ElbowFollower.GetEncoder()},
                                 m_BackLimit{m_ShoulderMotor.GetReverseLimitSwitch(rev::CANDigitalInput::LimitSwitchPolarity::kNormallyOpen)},
                                 m_FrontLimit{m_ShoulderMotor.GetForwardLimitSwitch(rev::CANDigitalInput::LimitSwitchPolarity::kNormallyOpen)},
                                 m_PDH{ArmConstants::PDH_ID, frc::PowerDistribution::ModuleType::kCTRE}
                                //  m_ElbowSolenoid{frc::PneumaticsModuleType::CTREPCM, ArmConstants::ElbowBrake1, ArmConstants::ElbowBrake2},
                                //  m_ShoulderSolenoid{frc::PneumaticsModuleType::CTREPCM, ArmConstants::ShoulderBrake1, ArmConstants::ShoulderBrake2}
{
    m_ShoulderMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
    m_ElbowMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
    m_ShoulderFollower.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
    m_ElbowFollower.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
    m_WristMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);

    m_ShoulderMotor.SetSmartCurrentLimit(15);
    m_ShoulderFollower.SetSmartCurrentLimit(15);

    m_ElbowMotor.SetSmartCurrentLimit(20);
    m_ElbowFollower.SetSmartCurrentLimit(20);

    m_WristMotor.SetSmartCurrentLimit(10);

    m_ShoulderFollower.Follow(m_ShoulderMotor);
    m_ElbowFollower.Follow(m_ElbowMotor);
    m_ElbowMotor.SetInverted(true);
    m_WristMotor.SetInverted(false);

    m_ElbowMotor.SetClosedLoopRampRate(0.00);
    m_ShoulderMotor.SetClosedLoopRampRate(0.00);
    m_ElbowMotor.SetOpenLoopRampRate(0.0);
    m_ShoulderMotor.SetOpenLoopRampRate(0.0);

    m_ShoulderPID.SetP(ArmConstants::kShoulderP, ArmConstants::kShoulderSlot);
    m_ShoulderPID.SetI(ArmConstants::kShoulderI, ArmConstants::kShoulderSlot);
    m_ShoulderPID.SetD(ArmConstants::kShoulderD, ArmConstants::kShoulderSlot);
    m_ShoulderPID.SetIZone(ArmConstants::kShoulderIZone, ArmConstants::kShoulderSlot);

    m_ElbowPID.SetP(ArmConstants::kElbowUpP, ArmConstants::kElbowUpSlot);
    m_ElbowPID.SetI(ArmConstants::kElbowDownI, ArmConstants::kElbowUpSlot);
    m_ElbowPID.SetD(ArmConstants::kElbowDownD, ArmConstants::kElbowUpSlot);
    m_ElbowPID.SetIZone(ArmConstants::kElbowIZone, ArmConstants::kElbowUpSlot);

    m_ElbowPID.SetP(ArmConstants::kElbowDownP, ArmConstants::kElbowDownSlot);
    m_ElbowPID.SetI(ArmConstants::kElbowDownI, ArmConstants::kElbowDownSlot);
    m_ElbowPID.SetD(ArmConstants::kElbowDownD, ArmConstants::kElbowDownSlot);
    m_ElbowPID.SetIZone(ArmConstants::kElbowIZone, ArmConstants::kElbowDownSlot);

    m_WristPID.SetP(ArmConstants::kWristP);
    m_WristPID.SetI(ArmConstants::kWristI);
    m_WristPID.SetD(ArmConstants::kWristD);

    m_ShoulderRelEncoder.SetPosition(0);
    m_ElbowRelEncoder.SetPosition(0);
    m_WristEncoder.SetPosition(-20.925);
}

double subsystem_Arm::CalculateShoulderAngle(double x, double y)
{
    double ElbowAngle = CalculateElbowAngle(x, y);
    return (atan(y / x) - atan((ArmConstants::ElbowJointLength * sin(ElbowAngle)) 
                                / (ArmConstants::ShoulderJointLength + (ArmConstants::ElbowJointLength * cos(ElbowAngle)))));
}

double subsystem_Arm::CalculateElbowAngle(double x, double y)
{
    return (-acos((pow(x, 2) + pow(y, 2) - pow(ArmConstants::ElbowJointLength, 2) - pow(ArmConstants::ShoulderJointLength, 2)) 
            / (2 * ArmConstants::ElbowJointLength * ArmConstants::ShoulderJointLength)));
}

void subsystem_Arm::SetElbowByPosition(double ElbowPos){
    DesiredElbowPosition = ElbowPos;
    SetShoulderPIDByDirection(DesiredElbowPosition);
}

void subsystem_Arm::SetShoulderByPosition(double ShoulderPos){
    DesiredShoulderPosition = ShoulderPos;
}

void subsystem_Arm::SetWristByPosition(double tiltPos){
    DesiredWristPostion = tiltPos;
    m_WristPID.SetReference(DesiredWristPostion, rev::ControlType::kPosition);
}

//WE ARE NOT USING THIS METHOD RN
// void subsystem_Arm::MoveArm(double x, double y)
// {
//     armX = x;
//     armY = y;
//     adjustedX = x - ArmConstants::xOriginAdjustment;
//     adjustedY = y - ArmConstants::yOriginAdjustment;

//     SetShoulderPIDByDirection(x, y);

//     if (adjustedX <= ArmConstants::totalArmLength && adjustedY <= ArmConstants::totalArmLength)
//     {
//         double Shoulder = CalculateShoulderAngle(adjustedX, adjustedY) - 180.0;
//         double Elbow = (Shoulder - CalculateElbowAngle(adjustedX, adjustedY));
//         convertedShoulder = Shoulder * ArmConstants::DegreesToRotations;
//         convertedElbow = Elbow * ArmConstants::DegreesToRotations;
//         frc::SmartDashboard::PutString("ARM MOVING", "ayush");
//         frc::SmartDashboard::PutNumber("Desired Shoulder Arm", convertedShoulder);
//         m_ShoulderPID.SetReference(convertedShoulder, rev::ControlType::kSmartMotion);
//         m_ElbowPID.SetReference(convertedElbow, rev::ControlType::kSmartMotion);
//     }
//     else
//     {
//         printf("INVALID X OR Y INPUT");
//     }
// }

// void subsystem_Arm::MoveArmManually(double leftAxis, double rightAxis)
// {
//     manualX = armX + (leftAxis * ArmConstants::JoystickToArm);
//     manualY = armY + (rightAxis * ArmConstants::JoystickToArm);
//     MoveArm(manualX, manualY);
// }

void subsystem_Arm::RunArmManualTest(double leftStick, double rightStick)
{
    DesiredShoulderPosition = ShoulderPosition + (leftStick * ArmConstants::JoystickToArm);
    DesiredElbowPosition = ElbowPosition + (rightStick * ArmConstants::JoystickToArm);
    SetShoulderPIDByDirection(DesiredShoulderPosition);
}


void subsystem_Arm::TiltWristManually(double trigger){
    DesiredWristPostion = WristPosition + trigger;
}
 
void subsystem_Arm::ManualMacroSwitch()
{
    m_IsManual = !m_IsManual;
}

bool subsystem_Arm::IsManual()
{
    return m_IsManual;
}

void subsystem_Arm::SetShoulderPIDByDirection(double Elbow)
{
    if (IsElbowDirectionGoingUp(Elbow))
    {
        m_ElbowMotor.SetSmartCurrentLimit(ArmConstants::ArmCurrentLimit);
        m_ElbowSlot = 0;
    }
    else
    {
        m_ElbowMotor.SetSmartCurrentLimit(ArmConstants::DownwardElbowCurrentLimit);
        m_ElbowSlot = 1;
    }
}

double subsystem_Arm::GetElbowPosition(){
    return ElbowPosition;
}

double subsystem_Arm::GetShoulderPosition(){
    return ShoulderPosition;
}

double subsystem_Arm::GetWristPosition(){
    return m_WristEncoder.GetPosition();
}

bool subsystem_Arm::IsElbowDirectionGoingUp(double Elbow)
{
    // double adjustedX = x + ArmConstants::xOriginAdjustment;
    // double adjustedY = y + ArmConstants::yOriginAdjustment;

    if(ElbowPosition < Elbow)
    {
        return true;
    }
    else if (ElbowPosition > Elbow)
    {
        return false;
    }
    return true;
}

bool subsystem_Arm::IsShoulderDirectionGoingUp(double x, double y)
{
    double adjustedX = x + ArmConstants::xOriginAdjustment;
    double adjustedY = y + ArmConstants::yOriginAdjustment;

    if (ShoulderPosition < (CalculateShoulderAngle(adjustedX, adjustedY) * ArmConstants::DegreesToRotations)) // Positive direction
    {
        return true;
    }
    else if (ShoulderPosition > (CalculateElbowAngle(adjustedX, adjustedY) * ArmConstants::DegreesToRotations)) // Negative Direction
    {
        return false;
    }
    return true;
}

bool subsystem_Arm::IsElbowAtDesiredPosition(){
    if(fabs(DesiredElbowPosition - ElbowPosition) < ArmConstants::bufferZone){
        return true;
    }
    return false;
}

bool subsystem_Arm::IsShoulderAtDesiredPosition(){
    if(fabs(DesiredShoulderPosition - ShoulderPosition) < ArmConstants::bufferZone){
        return true;
    }
    return false;
}

bool subsystem_Arm::IsWristAtDesiredPosition(){
    if(fabs(DesiredWristPostion - WristPosition) < ArmConstants::bufferZone){
        return true;
    }
    return false;
}

double subsystem_Arm::EncoderToDegrees(double ticks)
{
    return (ticks - ArmConstants::TicksOffset) * ArmConstants::TicksToDegrees;
}

void subsystem_Arm::Periodic()
{
    ShoulderPosition = m_ShoulderRelEncoder.GetPosition();
    ElbowPosition = m_ElbowRelEncoder.GetPosition();
    WristPosition = m_WristEncoder.GetPosition();

    ElbowAngle =   (m_ElbowRelEncoder.GetPosition() / 0.4444444 - 55);
    ShoulderAngle =   (m_ShoulderRelEncoder.GetPosition() / 0.44444 + 110);
    WristAngle =   (m_WristEncoder.GetPosition() / 0.2325  );


    double GravTorqueShoulder = 9.8 * ( cos(M_PI / 180.0 * ShoulderAngle) * ( ArmConstants::ShoulderJointMass * ArmConstants::ShoulderJointLength / 2.0 +
                                                                                ArmConstants::ElbowJointMass *  ArmConstants::ShoulderJointLength + 
                                                                                ArmConstants::IntakeJointMass * ArmConstants::ShoulderJointLength  ) 
                                            + cos(M_PI / 180.0 * ElbowAngle) * ( ArmConstants::ElbowJointMass * ArmConstants::ElbowJointLength / 2.0 +
                                                                                ArmConstants::IntakeJointMass * ArmConstants::ElbowJointLength )
                                            + cos(M_PI / 180.0 * WristAngle) * ( ArmConstants::IntakeJointMass * ArmConstants::IntakeJointLength / 2.0) );

    Power4Shoulder = GravTorqueShoulder / (40.0* 160);
    frc::SmartDashboard::PutNumber("Power4Shoulder", Power4Shoulder);

    Power4Elbow = 0.020 * cos(  M_PI / 180.0 * ElbowAngle);
    frc::SmartDashboard::PutNumber("Power4Elbow", Power4Elbow);

    // double Power4Wrist = Increment * cos( M_PI / 180.0 * WristAngle);
    // frc::SmartDashboard::PutNumber("Power4Wrist", Power4Wrist);

    m_ElbowPID.SetReference(DesiredElbowPosition, rev::ControlType::kPosition, m_ElbowSlot, Power4Elbow, rev::CANPIDController::ArbFFUnits::kPercentOut);
    m_ShoulderPID.SetReference(DesiredShoulderPosition, rev::ControlType::kPosition, 0, Power4Shoulder, rev::CANPIDController::ArbFFUnits::kPercentOut);
    m_WristPID.SetReference(DesiredWristPostion, rev::ControlType::kPosition, 0);

    //Potential Code for changing feedforward based on voltage reading

    // if( m_PDH.GetVoltage() > 12.6){
    //     Power4Elbow *= 0.99;
    // }else if( m_PDH.GetVoltage() < 11.5){
    //     Power4Elbow *= 1.01;
    // }

    frc::SmartDashboard::PutNumber("ElbowAngle", ElbowAngle);
    frc::SmartDashboard::PutNumber("ShoulderAngle", ShoulderAngle);
    frc::SmartDashboard::PutNumber("WristAngle", WristAngle);
    frc::SmartDashboard::PutNumber("Shoulder Arm Pos", ShoulderPosition);
    frc::SmartDashboard::PutNumber("Elbow Arm Pos", ElbowPosition);
    frc::SmartDashboard::PutNumber("Intake Tilt Pos", WristPosition);
    frc::SmartDashboard::PutNumber("Shoulder Arm Current", m_ShoulderMotor.GetOutputCurrent());
    frc::SmartDashboard::PutNumber("Elbow Arm Current", m_ElbowMotor.GetOutputCurrent());
}
