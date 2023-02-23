// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/command_SetToZero.h"

command_SetToZero::command_SetToZero(subsystem_Arm *arm) : m_arm{arm} {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements({m_arm});
}

// Called when the command is initially scheduled.
void command_SetToZero::Initialize() {
  m_arm->SetToZero();
}

// Called repeatedly when this Command is scheduled to run
void command_SetToZero::Execute() {}

// Called once the command ends or is interrupted.
void command_SetToZero::End(bool interrupted) {}

// Returns true when the command should end.
bool command_SetToZero::IsFinished() {
  return false;
}
