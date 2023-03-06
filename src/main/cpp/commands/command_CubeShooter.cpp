// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/command_CubeShooter.h"

command_CubeShooter::command_CubeShooter(subsystem_Intake *intake, std::function<double()> TriggerInput) : 
m_Intake{intake},
m_TriggerInput{TriggerInput} {
    AddRequirements({m_Arm});
    AddRequirements({m_PoseTracker});
    AddRequirements({m_Intake});

}

// Called when the command is initially scheduled.
void command_CubeShooter::Initialize() { 
    
    //Static velocities
    if(m_TriggerInput() > ControllerConstants::TriggerActivate ){
        m_Intake->SetHighCubeStaticVelocity();
    }else if( m_TriggerInput() < -ControllerConstants::TriggerActivate) {
        m_Intake->SetMidCubeStaticVelocity();
    }else{
        m_Intake->SetIntakeWheelsOff();
    }


    //dynamic velocities

}

// Called repeatedly when this Command is scheduled to run
void command_CubeShooter::Execute() {}

// Called once the command ends or is interrupted.
void command_CubeShooter::End(bool interrupted) {}

// Returns true when the command should end.
bool command_CubeShooter::IsFinished() {
  return true;
}
