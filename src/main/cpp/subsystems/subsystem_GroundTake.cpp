// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/subsystem_GroundTake.h"

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

}

// This method will be called once per scheduler run
void subsystem_GroundTake::Periodic() {
    
}
