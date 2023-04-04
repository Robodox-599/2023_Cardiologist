// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/command_ToggleGamePieceMode.h"

command_ToggleGamePieceMode::command_ToggleGamePieceMode(subsystem_Arm *Arm) : m_Arm{Arm} {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements({m_Arm});
}

// Called when the command is initially scheduled.
void command_ToggleGamePieceMode::Initialize() {
  m_Arm->ChangeGamePieceMode();
}

// Called repeatedly when this Command is scheduled to run
void command_ToggleGamePieceMode::Execute() {}

// Called once the command ends or is interrupted.
void command_ToggleGamePieceMode::End(bool interrupted) {}

// Returns true when the command should end.
bool command_ToggleGamePieceMode::IsFinished() {
  return false;
}
