// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/command_CubeShooter.h"

command_CubeShooter::command_CubeShooter(subsystem_Intake *Intake, std::function<bool()> IsHighNode) : 
m_Intake{Intake},
m_IsHighNode{IsHighNode},
m_Timer{} {

    AddRequirements({m_Intake});

}

// Called when the command is initially scheduled.
void command_CubeShooter::Initialize() { 
  m_Timer.Reset();
  m_Timer.Start();

  if(m_IsHighNode()){
    m_Intake->SetHighCubeStaticVelocity();
  }else{
    m_Intake->SetMidCubeStaticVelocity();
  }



}

// Called repeatedly when this Command is scheduled to run
void command_CubeShooter::Execute() {
  if(m_Intake->GetCurrentState() != IntakeConstants::State::Empty) {
    m_Timer.Reset();
    m_Timer.Start();
  }
}

// Called once the command ends or is interrupted.
void command_CubeShooter::End(bool interrupted) {
  m_Intake->SetPassive();
}

// Returns true when the command should end.
bool command_CubeShooter::IsFinished() {
  return m_Timer.Get() >= IntakeConstants::TimerConstant;
}
