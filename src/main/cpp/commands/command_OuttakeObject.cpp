// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/command_OuttakeObject.h"

command_OuttakeObject::command_OuttakeObject(subsystem_Intake* intake) : m_Intake{intake}, m_Timer{} {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements({intake});
}

// Called when the command is initially scheduled.
void command_OuttakeObject::Initialize() {
  m_Timer.Reset();
  m_Timer.Start();
  // if(m_Intake->GetCurrentState() == IntakeConstants::State::Purple){
  //   m_Intake->SetOutake();
  // }
  m_Intake->IntakeOpen();
}

// Called repeatedly when this Command is scheduled to run
void command_OuttakeObject::Execute() {
  if(m_Intake->GetCurrentState() != IntakeConstants::State::Empty) {
    m_Timer.Reset();
    m_Timer.Start();
  }
}

// Called once the command ends or is interrupted.
void command_OuttakeObject::End(bool interrupted) {
  m_Intake->SetPassive();
}

// Returns true when the command should end.
bool command_OuttakeObject::IsFinished() {
  return m_Timer.Get() >= IntakeConstants::TimerConstant;
}
