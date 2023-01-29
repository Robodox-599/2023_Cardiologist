// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/command_ResetModules.h"

command_ResetModules::command_ResetModules(subsystem_DriveTrain* DriveTrain): m_Drive{DriveTrain} {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements({m_Drive});
}

// Called when the command is initially scheduled.
void command_ResetModules::Initialize() {
  m_Drive->ResetModulesToAbsolute();
}

// Called repeatedly when this Command is scheduled to run
void command_ResetModules::Execute() {}

// Called once the command ends or is interrupted.
void command_ResetModules::End(bool interrupted) {}

// Returns true when the command should end.
bool command_ResetModules::IsFinished() {
  return true;
}
