// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/command_StartStationBalance.h"

command_StartStationBalance::command_StartStationBalance(subsystem_DriveTrain* Drive): m_Drive{Drive} {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements({m_Drive});
}

// Called when the command is initially scheduled.
void command_StartStationBalance::Initialize() {
  m_Drive->SetStationBalance();
}

// Called repeatedly when this Command is scheduled to run
void command_StartStationBalance::Execute() {}

// Called once the command ends or is interrupted.
void command_StartStationBalance::End(bool interrupted) {}

// Returns true when the command should end.
bool command_StartStationBalance::IsFinished() {
  return true;
}
