// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/command_Clamp.h"

command_Clamp::command_Clamp(subsystem_Intake* Intake): m_Intake{Intake} {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements({m_Intake});
}

// Called when the command is initially scheduled.
void command_Clamp::Initialize() {
  m_Intake->IntakeClose();
}

// Called repeatedly when this Command is scheduled to run
void command_Clamp::Execute() {}

// Called once the command ends or is interrupted.
void command_Clamp::End(bool interrupted) {}

// Returns true when the command should end.
bool command_Clamp::IsFinished() {
  return true;
}
