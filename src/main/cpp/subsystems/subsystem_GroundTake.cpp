// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/subsystem_GroundTake.h"
#include <frc/smartdashboard/SmartDashboard.h>

subsystem_GroundTake::subsystem_GroundTake(): 
                                              m_LeftBeamBreak{GroundTakeConstants::LeftBeamBreakID},
                                              m_CenterBeamBreak{GroundTakeConstants::CenterBeamBreakID},
                                              m_RightBeamBreak{GroundTakeConstants::RightBeamBreakID},
                                              m_ExtenderMotor{GroundTakeConstants::ExtenderID, rev::CANSparkMaxLowLevel::MotorType::kBrushless},
                                              m_IntakeMotor{GroundTakeConstants::IntakeID, rev::CANSparkMaxLowLevel::MotorType::kBrushless},
                                              m_ExtenderPID{m_ExtenderMotor.GetPIDController()},
                                              m_IntakePID{m_IntakeMotor.GetPIDController()},
                                              m_ExtenderRelEncoder{m_ExtenderMotor.GetEncoder()},
                                              m_IntakeRelEncoder{m_IntakeMotor.GetEncoder()} {
    m_IntakeMotor.SetInverted(false);
    m_IntakeMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    m_IntakeMotor.SetSmartCurrentLimit(21);

    // frc::SmartDashboard::PutNumber("kP", 0.0);
    // frc::SmartDashboard::PutNumber("kI", 0.0);
    // frc::SmartDashboard::PutNumber("kD", 0.0);
    // frc::SmartDashboard::PutNumber("kFF", 0.0);
    // frc::SmartDashboard::PutNumber("Vel", 0.0);
    // frc::SmartDashboard::PutNumber("Accel", 0.0);
    m_ExtenderMotor.SetInverted(false);
    m_ExtenderMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
    m_ExtenderMotor.SetSmartCurrentLimit(25);
    m_ExtenderPID.SetP(GroundTakeConstants::kP);
    m_ExtenderPID.SetI(0.0);
    m_ExtenderPID.SetD(GroundTakeConstants::kD);
    m_ExtenderPID.SetFF(GroundTakeConstants::kFF);
    m_ExtenderMotor.EnableSoftLimit(rev::CANSparkMax::SoftLimitDirection::kForward, true);
    m_ExtenderMotor.EnableSoftLimit(rev::CANSparkMax::SoftLimitDirection::kReverse, true);
    m_ExtenderPID.SetSmartMotionMaxVelocity(GroundTakeConstants::maxVel);
    m_ExtenderPID.SetSmartMotionMaxAccel(GroundTakeConstants::maxAccel);

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

void subsystem_GroundTake::RunIntake(){
    m_IntakeMotor.Set(0.4);
}

void subsystem_GroundTake::StopIntake(){
    m_IntakeMotor.Set(0.0);
}

frc2::CommandPtr subsystem_GroundTake::RunIntakeCommand(){
    return RunEnd([this]{return RunIntake();}, [this]{return StopIntake();});
}

frc2::CommandPtr subsystem_GroundTake::ExtendGroundTakeCommand(){
    return RunOnce([this]{return ExtendGroundTake();});
}

frc2::CommandPtr subsystem_GroundTake::RetractGroundTakeCommand(){
    return RunOnce([this]{return RetractGroundTake();});
}


// This method will be called once per scheduler run
void subsystem_GroundTake::Periodic() {
    frc::SmartDashboard::PutNumber("ExtenderPos", m_ExtenderRelEncoder.GetPosition());
    frc::SmartDashboard::PutBoolean("IsCubeDetected", IsCubeDetected());
    frc::SmartDashboard::PutNumber("LEFTBB", m_LeftBeamBreak.GetVoltage());
    frc::SmartDashboard::PutNumber("CenterBB", m_CenterBeamBreak.GetVoltage());
    frc::SmartDashboard::PutNumber("RightBB", m_RightBeamBreak.GetVoltage());



    
}
