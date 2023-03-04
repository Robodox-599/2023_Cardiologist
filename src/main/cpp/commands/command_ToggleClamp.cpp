// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/command_ToggleClamp.h"

command_ToggleClamp::command_ToggleClamp(subsystem_Intake* intake) : m_Intake{intake}, m_Timer{} {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements({m_Intake});
}

// Called when the command is initially scheduled.
void command_ToggleClamp::Initialize() {
  if(m_Intake->IsIntakeOpen()){
    m_Intake->SetPassive();
    m_Intake->IntakeClose();
    
  }else{
    m_Intake->SetOff();
    m_Intake->IntakeOpen();
  }
}

// Called repeatedly when this Command is scheduled to run
void command_ToggleClamp::Execute() {}

// Called once the command ends or is interrupted.
void command_ToggleClamp::End(bool interrupted) {
}

// Returns true when the command should end.
bool command_ToggleClamp::IsFinished() {
  return true;
}
