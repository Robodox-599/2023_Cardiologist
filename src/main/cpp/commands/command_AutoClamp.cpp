// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/command_AutoClamp.h"

command_AutoClamp::command_AutoClamp(subsystem_Intake* intake) : m_Intake{intake}, m_Timer{} {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements({intake});
}

// Called when the command is initially scheduled.
void command_AutoClamp::Initialize() {
  m_Intake->IntakeOpen();
  m_Timer.Reset();
  m_Timer.Start();
  // if(m_Intake->GetCurrentState() == IntakeConstants::State::Purple){
  //   m_Intake->SetOutake();
  // }
  // m_Intake->IntakeClose();
}

// Called repeatedly when this Command is scheduled to run
void command_AutoClamp::Execute() {
  if(m_Intake->GetCurrentProximity() >= 80.0) {
    m_Timer.Reset();
    m_Timer.Start();
  }
}

// Called once the command ends or is interrupted.
void command_AutoClamp::End(bool interrupted) {
  m_Intake->IntakeClose();
}

// Returns true when the command should end.
bool command_AutoClamp::IsFinished() {
  // return (m_Intake->GetCurrentState() != IntakeConstants::State::Empty);
  return m_Timer.Get() > IntakeConstants::TimerConstant;
}
