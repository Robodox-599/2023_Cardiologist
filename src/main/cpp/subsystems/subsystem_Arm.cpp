// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/subsystem_Arm.h"

subsystem_Arm::subsystem_Arm() : m_ShoulderMotor{ArmConstants::ShoulderMotorID, rev::CANSparkMaxLowLevel::MotorType::kBrushless},
                                 m_ShoulderFollower{ArmConstants::ShoulderFollowerID, rev::CANSparkMaxLowLevel::MotorType::kBrushless},
                                 m_ElbowMotor{ArmConstants::ElbowMotorID, rev::CANSparkMaxLowLevel::MotorType::kBrushless},
                                 m_ElbowFollower{ArmConstants::ElbowFollowerID, rev::CANSparkMaxLowLevel::MotorType::kBrushless},
                                 m_WristMotor{ArmConstants::WristMotorID, rev::CANSparkMaxLowLevel::MotorType::kBrushless},
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
                                 m_BackLimit{m_ShoulderMotor.GetForwardLimitSwitch(rev::SparkMaxLimitSwitch::Type::kNormallyOpen)},
                                 m_FrontLimit{m_ShoulderMotor.GetReverseLimitSwitch(rev::SparkMaxLimitSwitch::Type::kNormallyOpen)},
                                 m_PDH{ArmConstants::PDH_ID, frc::PowerDistribution::ModuleType::kCTRE},
                                 m_ElbowAbsEncoder{ArmConstants::ElbowAbsEncoderID},
                                 m_WristAbsEncoder{ArmConstants::WristAbsEncoderID},
                                 m_ElbowFeedforward{ArmConstants::kElbowS, ArmConstants::kElbowG, ArmConstants::kElbowV, ArmConstants::kElbowA},
                                 m_ShoulderFeedforward{ArmConstants::kShoulderS, ArmConstants::kShoulderG, ArmConstants::kShoulderV, ArmConstants::kShoulderA},
                                 m_WristFeedforward{ArmConstants::kWristS, ArmConstants::kWristG, ArmConstants::kWristV, ArmConstants::kWristA}
{
    m_ShoulderMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
    m_ElbowMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
    m_ShoulderFollower.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
    m_ElbowFollower.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
    m_WristMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);

    // m_ShoulderMotor.SetSmartCurrentLimit(10);
    // m_ShoulderFollower.SetSmartCurrentLimit(10);

    // Elbow current ends up getting redefined in the SetElbowPIDByDirection method
    // m_ElbowMotor.SetSmartCurrentLimit(25);
    // m_ElbowFollower.SetSmartCurrentLimit(25);

    m_WristMotor.SetSmartCurrentLimit(10);

    m_ShoulderFollower.Follow(m_ShoulderMotor);
    m_ElbowFollower.Follow(m_ElbowMotor);
    m_ElbowMotor.SetInverted(true);
    m_WristMotor.SetInverted(false);
    m_ShoulderMotor.SetInverted(true);

    m_ElbowMotor.SetClosedLoopRampRate(0.00);
    m_ShoulderMotor.SetClosedLoopRampRate(0.00);
    m_ElbowMotor.SetOpenLoopRampRate(0.0);
    m_ShoulderMotor.SetOpenLoopRampRate(0.0);

    m_ShoulderPID.SetP(0.1, 0);
    m_ShoulderPID.SetI(0, 0);
    m_ShoulderPID.SetD(0, 0);
    m_ShoulderPID.SetIZone(ArmConstants::kShoulderIZone, 0);

    m_ShoulderPID.SetP(0.05, 1);
    m_ShoulderPID.SetI(0, 1);
    m_ShoulderPID.SetD(0, 1);
    m_ShoulderPID.SetIZone(ArmConstants::kShoulderIZone, 1);  

    m_ElbowPID.SetP(0.02, ArmConstants::kElbowUpSlot);
    m_ElbowPID.SetI(0.0000, ArmConstants::kElbowUpSlot);
    m_ElbowPID.SetD(0.0, ArmConstants::kElbowUpSlot);
    m_ElbowPID.SetIZone(ArmConstants::kElbowIZone, ArmConstants::kElbowUpSlot);

    m_ElbowPID.SetP(0.02, ArmConstants::kElbowDownSlot);
    m_ElbowPID.SetI(0, ArmConstants::kElbowDownSlot);
    m_ElbowPID.SetD(0, ArmConstants::kElbowDownSlot);
    m_ElbowPID.SetIZone(ArmConstants::kElbowIZone, ArmConstants::kElbowDownSlot);

    m_WristPID.SetP(4.0);
    m_WristPID.SetI(ArmConstants::kWristI);
    m_WristPID.SetD(ArmConstants::kWristD);

    m_ShoulderRelEncoder.SetPosition(0);
     m_ElbowRelEncoder.SetPosition((m_ElbowAbsEncoder.GetAbsolutePosition() - 0.77) * ArmConstants::kElbowGearRatio);
    // m_ElbowRelEncoder.SetPosition(0);
    m_WristEncoder.SetPosition(-20.925);

    m_ElbowMotor.EnableSoftLimit(rev::CANSparkMax::SoftLimitDirection::kForward, true);
    m_ElbowMotor.EnableSoftLimit(rev::CANSparkMax::SoftLimitDirection::kReverse, true);
    m_WristMotor.EnableSoftLimit(rev::CANSparkMax::SoftLimitDirection::kForward, false);
    m_WristMotor.EnableSoftLimit(rev::CANSparkMax::SoftLimitDirection::kReverse, false);
    m_ShoulderMotor.EnableSoftLimit(rev::CANSparkMax::SoftLimitDirection::kForward, true);
    m_ShoulderMotor.EnableSoftLimit(rev::CANSparkMax::SoftLimitDirection::kReverse, true);

    m_ElbowMotor.SetSoftLimit(rev::CANSparkMax::SoftLimitDirection::kForward, 33.5);
    m_ElbowMotor.SetSoftLimit(rev::CANSparkMax::SoftLimitDirection::kReverse, -25.0);
    m_WristMotor.SetSoftLimit(rev::CANSparkMax::SoftLimitDirection::kForward, 14.0);
     m_WristMotor.SetSoftLimit(rev::CANSparkMax::SoftLimitDirection::kReverse, -30.0);
    m_ShoulderMotor.SetSoftLimit(rev::CANSparkMax::SoftLimitDirection::kForward, 30.0);
    m_ShoulderMotor.SetSoftLimit(rev::CANSparkMax::SoftLimitDirection::kReverse, 0.0);

    m_ElbowPID.SetSmartMotionMaxAccel(60, 0);
    m_ElbowPID.SetSmartMotionMaxVelocity(60, 0);
    m_ElbowPID.SetSmartMotionMinOutputVelocity(60, 0);

    m_ElbowPID.SetSmartMotionMaxAccel(60, 1);
    m_ElbowPID.SetSmartMotionMaxVelocity(60, 1);
    m_ElbowPID.SetSmartMotionMinOutputVelocity(60, 1);

    m_FrontLimit.EnableLimitSwitch(true);
    m_BackLimit.EnableLimitSwitch(true);

    // m_WristTimer.Start();
    // m_ElbowTimer.Start();
    // m_ShoulderTimer.Start(); 
}

frc2::CommandPtr subsystem_Arm::MoveShoulderCommand(double EncPosition){
    return RunOnce([this, EncPosition]{return SetShoulderByPosition(EncPosition);});
}
frc2::CommandPtr subsystem_Arm::MoveElbowCommand(double EncPosition){
    return RunOnce([this, EncPosition]{return SetElbowByPosition(EncPosition);});

}
frc2::CommandPtr subsystem_Arm::MoveWristCommand(double EncPosition){
    return RunOnce([this, EncPosition]{return SetWristByPosition(EncPosition);});
}

frc2::CommandPtr subsystem_Arm::ToHighCone(){
    return frc2::cmd::Sequence(
    frc2::cmd::Deadline(frc2::WaitUntilCommand([this]{return WristThreshold(15 * 0.80);}).ToPtr(),
                    MoveWristCommand(15)), 
    frc2::cmd::Deadline(frc2::WaitUntilCommand([this]{return ElbowThreshold(ArmConstants::HighConeElbow * 0.80);}).ToPtr(),
                    MoveElbowCommand(ArmConstants::HighConeElbow)),  
    MoveWristCommand(ArmConstants::HighConeTilt),
    MoveShoulderCommand(ArmConstants::HighConeShoulder)           
    );
}

frc2::CommandPtr subsystem_Arm::ToMidCone(){
    return frc2::cmd::Sequence(
    frc2::cmd::Deadline(frc2::WaitUntilCommand([this]{return WristThreshold(15 * 0.80);}).ToPtr(),
                    MoveWristCommand(15)), 
    frc2::cmd::Deadline(frc2::WaitUntilCommand([this]{return ElbowThreshold(ArmConstants::MidConeElbow * 0.50);}).ToPtr(),
                    MoveElbowCommand(ArmConstants::MidConeElbow)),  
    MoveWristCommand(ArmConstants::MidConeTilt),
    MoveShoulderCommand(ArmConstants::MidConeShoulder)           
    );
}


frc2::CommandPtr subsystem_Arm::ToHighCube(){
    return frc2::cmd::Sequence(
    frc2::cmd::Deadline(frc2::WaitUntilCommand([this]{return WristThreshold(15 * 0.80);}).ToPtr(),
                    MoveWristCommand(15)), 
    frc2::cmd::Deadline(frc2::WaitUntilCommand([this]{return ElbowThreshold(ArmConstants::HighCubeElbow * 0.80);}).ToPtr(),
                    MoveElbowCommand(ArmConstants::HighCubeElbow)),  
    MoveShoulderCommand(ArmConstants::HighConeShoulder),
    frc2::cmd::Deadline(frc2::WaitUntilCommand([this]{return WristThreshold(ArmConstants::HighCubeTilt);}).ToPtr(),
                    MoveWristCommand(ArmConstants::HighCubeTilt))
    );
}

frc2::CommandPtr subsystem_Arm::ToMidCube(){
    return frc2::cmd::Sequence(
    frc2::cmd::Deadline(frc2::WaitUntilCommand([this]{return WristThreshold(15 * 0.80);}).ToPtr(),
                    MoveWristCommand(15)), 
    frc2::cmd::Deadline(frc2::WaitUntilCommand([this]{return ElbowThreshold(ArmConstants::MidCubeElbow * 0.60);}).ToPtr(),
                    MoveElbowCommand(ArmConstants::MidCubeElbow)),  
    MoveShoulderCommand(ArmConstants::MidCubeShoulder),
    frc2::cmd::Deadline(frc2::WaitUntilCommand([this]{return WristThreshold(ArmConstants::MidCubeTilt * 0.50);}).ToPtr(),
                    MoveWristCommand(ArmConstants::MidCubeTilt))
    );   
}

frc2::CommandPtr subsystem_Arm::ToTiltedStow(){
    return frc2::cmd::Sequence(
    MoveWristCommand(14), 
    MoveShoulderCommand(ArmConstants::TiltedStowShoulder),
    MoveElbowCommand(ArmConstants::TiltedStowElbow),
    MoveWristCommand(ArmConstants::ScoreTilt)
    );  
}

void subsystem_Arm::PollArmPosition(int POV){
    switch(POV){
        case 0:
            m_ArmPoll = DPAD::NODE_LEVEL::HIGH;
            break;
        case 270:
            m_ArmPoll = DPAD::NODE_LEVEL::MID;
            break;
        case 180:
            m_ArmPoll = DPAD::NODE_LEVEL::LOW;
            break;
        default:
            m_ArmPoll = DPAD::NODE_LEVEL::NON_SPECIFIED;
            break;
    }
}

frc2::CommandPtr subsystem_Arm::ConeMovement(){
    switch(m_ArmPoll){
        case DPAD::NODE_LEVEL::HIGH:
            return ToHighCone();
            break;
        case DPAD::NODE_LEVEL::MID:
            return ToMidCone();
            break;
        case DPAD::NODE_LEVEL::LOW:
            return ToTiltedStow();
            break;
        default:
            return RunOnce([this]{frc2::WaitCommand(0.0_s);});
            break;
    }
}

frc2::CommandPtr subsystem_Arm::CubeMovement(){
    switch(m_ArmPoll){
        case DPAD::NODE_LEVEL::HIGH:
            return ToHighCube();
            break;
        case DPAD::NODE_LEVEL::MID:
            return ToMidCube();
            break;
        case DPAD::NODE_LEVEL::LOW:
            return ToTiltedStow();
            break;
        default:
            return RunOnce([this]{frc2::WaitCommand(0.0_s);});
            break;
    }
}

DPAD::NODE_LEVEL subsystem_Arm::GetArmPoll(){
    return m_ArmPoll;
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
    SetElbowPIDByDirection(DesiredElbowPosition);
    // m_ElbowPID.SetReference(DesiredElbowPosition, rev::ControlType::kSmartMotion, m_ElbowSlot);
}

void subsystem_Arm::SetShoulderByPosition(double ShoulderPos){
    DesiredShoulderPosition = ShoulderPos;
    SetShoulderPIDByDirection(DesiredShoulderPosition);
    // m_ShoulderPID.SetReference(DesiredShoulderPosition, rev::ControlType::kSmartMotion, m_ShoulderSlot);
}

void subsystem_Arm::SetWristByPosition(double tiltPos){
    DesiredWristPostion = tiltPos;
}

//WE ARE NOT USING THIS METHOD RN
// void subsystem_Arm::MoveArm(double x, double y)
// {
//     armX = x;
//     armY = y;
//     adjustedX = x - ArmConstants::xOriginAdjustment;
//     adjustedY = y - ArmConstants::yOriginAdjustment;

//     SetElbowPIDByDirection(x, y);

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
    DesiredShoulderPosition += (leftStick * ArmConstants::JoystickToArm);
    DesiredElbowPosition -= (rightStick * ArmConstants::JoystickToArm);
    SetElbowPIDByDirection(DesiredShoulderPosition);
}


void subsystem_Arm::TiltWristManually(double trigger){
    DesiredWristPostion +=  trigger * ArmConstants::TriggerToArm;

}

frc2::CommandPtr subsystem_Arm::ResetWrist(){

    return RunOnce([this]{return m_WristEncoder.SetPosition(0.0);});
}

void subsystem_Arm::SetElbowPIDByDirection(double Elbow)
{
    if (IsElbowDirectionGoingUp(Elbow))
    {
        m_ElbowMotor.SetSmartCurrentLimit(25);
        m_ElbowSlot = 0;
    }
    else
    {
        m_ElbowMotor.SetSmartCurrentLimit(25);
        m_ElbowSlot = 1;
    }
}

void subsystem_Arm::SetShoulderPIDByDirection(double desiredShoulderPos){
    if(IsShoulderDirectionGoingUp(desiredShoulderPos)){
        if(desiredShoulderPos < 20.0){
            m_ShoulderMotor.SetSmartCurrentLimit(30);
        } else {
            m_ShoulderMotor.SetSmartCurrentLimit(30);
        }
        m_ShoulderSlot = 1;
    }
    else{
        m_ShoulderMotor.SetSmartCurrentLimit(30);
        m_ShoulderSlot = 0;
    }
}

double subsystem_Arm::GetElbowPosition(){
    return ElbowPosition;
}

double subsystem_Arm::GetShoulderPosition(){
    return ShoulderPosition;
}

double subsystem_Arm::GetWristPosition(){
    return WristPosition;
}

bool subsystem_Arm::IsElbowDirectionGoingUp(double Elbow)
{
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

bool subsystem_Arm::IsShoulderDirectionGoingUp(double shoulder)
{
    if (m_ShoulderRelEncoder.GetPosition() > shoulder) 
    {
        return true;
    }
    else if (m_ShoulderRelEncoder.GetPosition() < shoulder) 
    {
        return false;
    }
    return true;
}

bool subsystem_Arm::IsElbowAtDesiredPosition(){
    if(fabs(DesiredElbowPosition - ElbowEnc) < ArmConstants::bufferZone){
        return true;
    }
    return false;
}

bool subsystem_Arm::IsShoulderAtDesiredPosition(){
    if(fabs(DesiredShoulderPosition - ShoulderEnc) < ArmConstants::bufferZone){
        return true;
    }
    return false;
}

bool subsystem_Arm::IsWristAtDesiredPosition(){
    if(fabs(DesiredWristPostion - WristEnc) < ArmConstants::bufferZone){
        return true;
    }
    return false;
}

double subsystem_Arm::GetShoulderIncrement(){

    double Difference = (DesiredShoulderPosition - ShoulderPosition);
    // frc::SmartDashboard::PutBoolean("IsShoulderCrawl", false);

    
    if( fabs(Difference) > ArmConstants::ErrorBound){   
        // frc::SmartDashboard::PutBoolean("IsShoulderCrawl", false);

        if( Difference > ArmConstants::kShoulderStep ){

            return ShoulderPosition + ArmConstants::kShoulderStep;
        }else if( Difference < -ArmConstants::kShoulderStep ){
            return ShoulderPosition - ArmConstants::kShoulderStep;
        }else{
            return DesiredShoulderPosition;
        }
    }else{
        // frc::SmartDashboard::PutBoolean("IsShoulderCrawl", true);

        if( Difference > ArmConstants::kShoulderCrawl ){
            return ShoulderPosition + ArmConstants::kShoulderCrawl;
        }else if( Difference < -ArmConstants::kShoulderCrawl ){
            return ShoulderPosition - ArmConstants::kShoulderCrawl;
        }else{
            return DesiredShoulderPosition;
        }   
    }
    
}

double subsystem_Arm::GetWristIncrement(){
        double Difference = (DesiredWristPostion - WristPosition);

    if( fabs(Difference) > ArmConstants::ErrorBound){   
        if( Difference > ArmConstants::kWristStep ){

            return WristPosition + ArmConstants::kWristStep;
        }else if( Difference < -ArmConstants::kWristStep ){
            return WristPosition - ArmConstants::kWristStep;
        }else{
            return DesiredWristPostion;
        }
    }
    return 0.0;
}

double subsystem_Arm::GetElbowIncrement(){
        double Difference = (DesiredElbowPosition - ElbowPosition);
    // frc::SmartDashboard::PutBoolean("IsElbowCrawl", false);
    if( fabs(Difference) > ArmConstants::ErrorBound){  
        // frc::SmartDashboard::PutBoolean("IsElbowCrawl", false);
        if( Difference > ArmConstants::kElbowStep ){

            return ElbowPosition + ArmConstants::kElbowStep;
        }else if( Difference < -ArmConstants::kElbowStep ){
            return ElbowPosition - ArmConstants::kElbowStep;
        }else{
            return DesiredElbowPosition;
        }
    }else{
        // frc::SmartDashboard::PutBoolean("IsElbowCrawl", true);

        if( fabs(Difference) > ArmConstants::kElbowCrawl ){
            return ElbowPosition + ArmConstants::kElbowCrawl;
        }else if( Difference < -ArmConstants::kElbowCrawl ){
            return ElbowPosition - ArmConstants::kElbowCrawl;
        }else{
            return DesiredElbowPosition;
        }   
    }
}

units::angle::radian_t subsystem_Arm::RotationsToRadians(double rotations){
    units::angle::radian_t radians{rotations * ArmConstants::RotToRad};
    return radians;
}

bool subsystem_Arm::ElbowThreshold(double Threshold){
    if(m_ElbowSlot == 0){
        return ElbowEnc > Threshold;
    } else {
        return ElbowEnc < Threshold;
    }
}

bool subsystem_Arm::ShoulderThreshold(double Threshold){
    if(m_ShoulderSlot == 0){
        return ShoulderEnc > Threshold;
    } else {
        return ShoulderEnc < Threshold;
    }
}

bool subsystem_Arm::WristThreshold(double Threshold){
    if(DesiredWristPostion >= WristEnc){
        return WristEnc > Threshold;
    } else{
        return WristEnc < Threshold;
    }
}

void subsystem_Arm::Periodic()
{

    double relWristAngle = 180 + ( (m_WristAbsEncoder.GetAbsolutePosition() - 0.468) * 83.7 )/ 0.2325;
    double AbsWristAngle = 180 - (relWristAngle - ElbowAngle);

    double AbsWristPos = AbsWristAngle  * 83.7 / 360;
    m_WristEncoder.SetPosition(AbsWristPos);


    ShoulderEnc = m_ShoulderRelEncoder.GetPosition();
    ElbowEnc = m_ElbowRelEncoder.GetPosition();
    WristEnc = m_WristEncoder.GetPosition();

    // m_WristEncoder.SetPosition((m_WristAbsEncoder.GetAbsolutePosition() - 0.4663) * 83.7);
    

    ElbowAngle =   (m_ElbowRelEncoder.GetPosition() / (ArmConstants::kElbowGearRatio / 360.0) - 49);
    ShoulderAngle =   (m_ShoulderRelEncoder.GetPosition() / (ArmConstants::kElbowGearRatio / 360.0) + 110);
    WristAngle =   (m_WristEncoder.GetPosition() / 0.2325  );

    double GravTorqueShoulder = 9.8 * ( cos(M_PI / 180.0 * ShoulderAngle) * ( ArmConstants::ShoulderJointMass * ArmConstants::ShoulderJointLength / 2.0 +
                                                                                ArmConstants::ElbowJointMass *  ArmConstants::ShoulderJointLength + 
                                                                                ArmConstants::IntakeJointMass * ArmConstants::ShoulderJointLength  ) 
                                            + cos(M_PI / 180.0 * ElbowAngle) * ( ArmConstants::ElbowJointMass * ArmConstants::ElbowJointLength / 2.0 +
                                                                                ArmConstants::IntakeJointMass * ArmConstants::ElbowJointLength )
                                            + cos(M_PI / 180.0 * WristAngle) * ( ArmConstants::IntakeJointMass * ArmConstants::IntakeJointLength / 2.0) );

    Power4Shoulder = GravTorqueShoulder / (38.0 * 161.577);
    Power4Elbow = 0.012 * cos(  M_PI / 180.0 * ElbowAngle);


// :)

    DesiredElbowRadians = RotationsToRadians(DesiredElbowPosition);
    DesiredShoulderRadians = units::radian_t{ 1.88-( (2 * 3.14)/ (ArmConstants::kElbowGearRatio) * DesiredShoulderPosition )};
    DesiredWristRadians = units::angle::radian_t{DesiredWristPostion * 0.07197237113};

    // frc::SmartDashboard::PutNumber("ShoulderRadians", RotationsToRadians(m_ShoulderRelEncoder.GetPosition()).value());
    // if(DesiredElbowPosition >= ElbowEnc){
    //     ElbowFF = m_ElbowFeedforward.Calculate((DesiredElbowRadians - units::radian_t{0.45378}), ArmConstants::kEndVel, units::angular_acceleration::degrees_per_second_squared_t{2.5});
    // } else if (DesiredElbowPosition < ElbowEnc){
    //     ElbowFF = m_ElbowFeedforward.Calculate((DesiredElbowRadians - units::radian_t{0.45378}), ArmConstants::kEndVel, units::angular_acceleration::degrees_per_second_squared_t{5.0});
    // }
    ElbowFF = m_ElbowFeedforward.Calculate((DesiredElbowRadians - units::radian_t{0.45378}), ArmConstants::kEndVel, units::angular_acceleration::degrees_per_second_squared_t{1});

    ShoulderFF = m_ShoulderFeedforward.Calculate((DesiredShoulderRadians + units::radian_t{1.24965}), ArmConstants::kEndVel, units::angular_acceleration::degrees_per_second_squared_t{2.5});
    // WristFF = m_WristFeedforward.Calculate((DesiredShoulderRadians), ArmConstants::kEndVel, units::angular_acceleration::degrees_per_second_squared_t{6.0});
    m_ElbowPID.SetFF(ElbowFF.value(), m_ElbowSlot);
    m_ShoulderPID.SetFF(ShoulderFF.value(), m_ShoulderSlot);
    // m_WristPID.SetFF(WristFF.value(), 0);

    m_ElbowPID.SetReference(DesiredElbowPosition, rev::CANSparkMax::ControlType::kPosition, m_ElbowSlot);
    m_ShoulderPID.SetReference(DesiredShoulderPosition, rev::CANSparkMax::ControlType::kPosition, m_ShoulderSlot);
    m_WristPID.SetReference(DesiredWristPostion, rev::CANSparkMaxLowLevel::ControlType::kPosition, 0);

    // frc::SmartDashboard::PutNumber("DesiredShoulderPos", DesiredShoulderPosition);
    // m_ElbowPID.SetReference(ElbowPosition, rev::CANSparkMaxLowLevel::ControlType::kPosition, m_ElbowSlot, Power4Elbow, rev::SparkMaxPIDController::ArbFFUnits::kPercentOut);
    // m_ShoulderPID.SetReference(DesiredShoulderPosition, rev::CANSparkMaxLowLevel::ControlType::kPosition, m_ShoulderSlot, Power4Shoulder, rev::SparkMaxPIDController::ArbFFUnits::kPercentOut);

    // m_ElbowMotor.Set(Power4Elbow);
    // m_ShoulderMotor.Set(Power4Shoulder);

    // frc::SmartDashboard::PutBoolean("IsElbowDesired", IsElbowAtDesiredPosition());
    // frc::SmartDashboard::PutBoolean("IsShoulderDesired", IsShoulderAtDesiredPosition());
    frc::SmartDashboard::PutNumber("Shoulder Arm Pos", m_ShoulderRelEncoder.GetPosition());
    frc::SmartDashboard::PutNumber("Elbow Arm Pos", m_ElbowRelEncoder.GetPosition());
    frc::SmartDashboard::PutNumber("Intake Tilt Pos", m_WristEncoder.GetPosition());

}

