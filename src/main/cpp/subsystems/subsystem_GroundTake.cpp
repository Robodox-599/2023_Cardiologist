// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/subsystem_GroundTake.h"
#include <frc/smartdashboard/SmartDashboard.h>

subsystem_GroundTake::subsystem_GroundTake(): 
                                              m_LeftBeamBreak{GroundTakeConstants::LeftBeamBreakID},
                                            //   m_CenterBeamBreak{GroundTakeConstants::CenterBeamBreakID},
                                            //   m_RightBeamBreak{GroundTakeConstants::RightBeamBreakID},
                                              m_ExtenderMotor{GroundTakeConstants::ExtenderID, rev::CANSparkMaxLowLevel::MotorType::kBrushless},
                                              m_IntakeMotor{GroundTakeConstants::IntakeID, rev::CANSparkMaxLowLevel::MotorType::kBrushless},
                                              m_ExtenderPID{m_ExtenderMotor.GetPIDController()},
                                              m_IntakePID{m_IntakeMotor.GetPIDController()},
                                              m_ExtenderRelEncoder{m_ExtenderMotor.GetEncoder()},
                                              m_IntakeRelEncoder{m_IntakeMotor.GetEncoder()} {
    m_IntakeMotor.SetInverted(false);
    m_IntakeMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    m_IntakeMotor.SetSmartCurrentLimit(30);
    frc::SmartDashboard::PutNumber("InnerVel", 0.0);
    frc::SmartDashboard::PutNumber("OutterVel", 0.0);
    frc::SmartDashboard::PutNumber("Intake", 0.0);
    frc::SmartDashboard::GetNumber("InnerVel", 0.0);
    frc::SmartDashboard::GetNumber("OutterVel", 0.0);
    frc::SmartDashboard::GetNumber("Intake", 0.0);

    m_ExtenderMotor.SetInverted(false);
    m_ExtenderMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    m_ExtenderMotor.SetSmartCurrentLimit(25);
    m_ExtenderPID.SetP(GroundTakeConstants::kP);
    m_ExtenderPID.SetI(0.0);
    m_ExtenderPID.SetD(GroundTakeConstants::kD);
    m_ExtenderPID.SetFF(GroundTakeConstants::kFF);
    m_ExtenderMotor.EnableSoftLimit(rev::CANSparkMax::SoftLimitDirection::kForward, true);
    m_ExtenderMotor.EnableSoftLimit(rev::CANSparkMax::SoftLimitDirection::kReverse, true);
    m_ExtenderPID.SetSmartMotionMaxVelocity(GroundTakeConstants::maxVel);
    m_ExtenderPID.SetSmartMotionMaxAccel(GroundTakeConstants::maxAccel);


    m_IntakePID.SetP(0.00015);
    m_IntakePID.SetI(0.0);
    m_IntakePID.SetD(0.0);
    m_IntakePID.SetFF(0.0002);

    m_ExtenderRelEncoder.SetPosition(0.0);

    m_ExtenderMotor.SetSoftLimit(rev::CANSparkMax::SoftLimitDirection::kForward, 6.3);
    m_ExtenderMotor.SetSoftLimit(rev::CANSparkMax::SoftLimitDirection::kReverse, -1);    

    m_IntakeMotor.EnableSoftLimit(rev::CANSparkMax::SoftLimitDirection::kForward, false);
    m_IntakeMotor.EnableSoftLimit(rev::CANSparkMax::SoftLimitDirection::kReverse, false);

}

bool subsystem_GroundTake::IsCubeDetected(){
    // return ( m_LeftBeamBreak.GetVoltage() < 1 
    //         m_CenterBeamBreak.GetVoltage() < 1 ||
    //         m_RightBeamBreak.GetVoltage() < 1);
    return m_LeftBeamBreak.GetVoltage() < 1;
}

void subsystem_GroundTake::ExtendGroundTake(){
    m_ExtenderPID.SetReference(GroundTakeConstants::ExtendedPosition, rev::CANSparkMax::ControlType::kSmartMotion);

}

void subsystem_GroundTake::RetractGroundTake(){
    m_ExtenderPID.SetReference(GroundTakeConstants::RetractedPosition, rev::CANSparkMax::ControlType::kSmartMotion);
}

double subsystem_GroundTake::GetExtenderPosition(){
    return m_ExtenderRelEncoder.GetPosition();
}

void subsystem_GroundTake::RunPassThroughIntake(){
    m_IntakePower = GroundTakeConstants::Power::PassThroughIntake;
}

void subsystem_GroundTake::RunHybridIntake(){
    m_IntakePower = GroundTakeConstants::Power::HybridIntake;
}

void subsystem_GroundTake::StopIntake(){
    m_IntakePower = GroundTakeConstants::Power::Stopped;
}

void subsystem_GroundTake::StopAndRetract(){
    StopIntake();
    RetractGroundTake();
}

void subsystem_GroundTake::PassiveTransitionIntake(){
    m_IntakePower = GroundTakeConstants::Power::Transition;
}

void subsystem_GroundTake::Outake(){
    ExtendGroundTake();
    m_IntakePower = GroundTakeConstants::Power::Outake;
}

void subsystem_GroundTake::MaintainIntakeMode(){
    //  frc::SmartDashboard::GetNumber("InnerVel", 0.0);
    //frc::SmartDashboard::GetNumber("OutterVel", 0.0);
    //frc::SmartDashboard::GetNumber("Intake", 0.0);
    switch(m_IntakePower){
        case(GroundTakeConstants::Power::PassThroughIntake):
            // m_IntakeMotor.Set(0.4);
            m_IntakePID.SetReference(2500, rev::CANSparkMax::ControlType::kVelocity);
            break;
        case(GroundTakeConstants::Power::HybridIntake):
            m_IntakePID.SetReference(2000, rev::CANSparkMax::ControlType::kVelocity);
            break;
        case(GroundTakeConstants::Power::Transition):
            if(GetExtenderPosition() < GroundTakeConstants::ExtendedPosition * 0.75){
                // m_IntakeMotor.Set(-0.05);
                m_IntakePID.SetReference(-600, rev::CANSparkMax::ControlType::kVelocity);
            }else{
                // m_IntakeMotor.Set(0.015);
                m_IntakePID.SetReference(500, rev::CANSparkMax::ControlType::kVelocity);

            }
            break;
        case(GroundTakeConstants::Power::Stopped):
            m_IntakeMotor.Set(0.0);
            break;
        case(GroundTakeConstants::Power::Outake):
            m_IntakePID.SetReference(-2000, rev::CANSparkMax::ControlType::kVelocity);
            break;
    };
}

frc2::CommandPtr subsystem_GroundTake::StowCompleteleyCommand(){
    return RunOnce([this]{return StopAndRetract();});
}

frc2::CommandPtr subsystem_GroundTake::RunIntakeToggleCommand(){
    return RunEnd([this]{return RunHybridIntake();}, [this]{return StopAndRetract();});
}

frc2::CommandPtr subsystem_GroundTake::RunPassThroughIntakeCommand(){
    return RunOnce([this]{return RunPassThroughIntake();});
}

frc2::CommandPtr subsystem_GroundTake::RunHybridIntakeCommand(){
    return RunOnce([this]{return RunHybridIntake();});

}

frc2::CommandPtr subsystem_GroundTake::ExtendGroundTakeCommand(){
    return RunOnce([this]{return ExtendGroundTake();});
}

frc2::CommandPtr subsystem_GroundTake::RetractGroundTakeCommand(){
    return RunOnce([this]{return RetractGroundTake();});
}

frc2::CommandPtr subsystem_GroundTake::WaitUntilRetractedCommand(){
    return frc2::WaitUntilCommand([this]{return GetExtenderPosition() < GroundTakeConstants::ExtendedPosition * 0.25;}).ToPtr();
}

frc2::CommandPtr subsystem_GroundTake::WaitUntilEmptyCommand(){
    return frc2::WaitUntilCommand([this]{return !IsCubeDetected();}).ToPtr();
}

frc2::CommandPtr subsystem_GroundTake::StopIntakeCommand(){
    return RunOnce([this]{return StopIntake();});
}

frc2::CommandPtr subsystem_GroundTake::OutakeCommand(){
    return RunOnce([this]{return Outake();});
}




// This method will be called once per scheduler run
void subsystem_GroundTake::Periodic() {
    frc::SmartDashboard::PutNumber("ExtenderPos", m_ExtenderRelEncoder.GetPosition());
    frc::SmartDashboard::PutBoolean("IsCubeDetected", IsCubeDetected());
    // frc::SmartDashboard::PutNumber("LEFTBB", m_LeftBeamBreak.GetVoltage());
    // frc::SmartDashboard::PutNumber("CenterBB", m_CenterBeamBreak.GetVoltage());
    // frc::SmartDashboard::PutNumber("RightBB", m_RightBeamBreak.GetVoltage());
    MaintainIntakeMode();

    // frc::SmartDashboard::PutNumber("IntakeRelVel", m_IntakeRelEncoder.GetVelocity());


}
