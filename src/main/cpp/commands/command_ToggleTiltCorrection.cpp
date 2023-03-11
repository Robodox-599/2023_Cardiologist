// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/command_ToggleTiltCorrection.h"

command_ToggleTiltCorrection::command_ToggleTiltCorrection(subsystem_DriveTrain* DriveTrain):m_DriveTrain{DriveTrain} {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements({DriveTrain});
}

// Called when the command is initially scheduled.
void command_ToggleTiltCorrection::Initialize() {
  m_DriveTrain->ToggleBalanceCorrection();
}

// Called repeatedly when this Command is scheduled to run
void command_ToggleTiltCorrection::Execute() {}

// Called once the command ends or is interrupted.
void command_ToggleTiltCorrection::End(bool interrupted) {}

// Returns true when the command should end.
bool command_ToggleTiltCorrection::IsFinished() {
  return true;
}
