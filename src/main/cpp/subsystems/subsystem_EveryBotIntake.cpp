// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/subsystem_EveryBotIntake.h"

subsystem_EveryBotIntake::subsystem_EveryBotIntake(): m_IntakeMotor{IntakeConstants::IntakeMotorID, rev::CANSparkMaxLowLevel::MotorType::kBrushless},
                                                        m_IntakeMotorPID{m_IntakeMotor.GetPIDController()},
                                                        m_IntakeEncoder{m_IntakeMotor.GetEncoder()}
{
    m_IntakeMotor.SetSmartCurrentLimit(25);

}

void subsystem_EveryBotIntake::SetOff(){
    m_IntakeMotor.Set(0.0);
}

void subsystem_EveryBotIntake::SetIntake(){
    m_IntakeMotor.Set(IntakeConstants::IntakePower);
}

void subsystem_EveryBotIntake::SetOutake(){
    m_IntakeMotor.Set(IntakeConstants::OuttakePower);
}





// This method will be called once per scheduler run
void subsystem_EveryBotIntake::Periodic() {
    if(m_IntakeMotor.GetOutputCurrent() > 20){
        m_IntakeMotor.Set(0.0);
    }
}
