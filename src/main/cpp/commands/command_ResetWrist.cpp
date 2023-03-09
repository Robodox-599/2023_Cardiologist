// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/command_ResetWrist.h"

command_ResetWrist::command_ResetWrist(subsystem_Arm* Arm): m_Arm{Arm} {

  // Use addRequirements() here to declare subsystem dependencies.

  AddRequirements({m_Arm});
}

// Called when the command is initially scheduled.
void command_ResetWrist::Initialize() {
  m_Arm->ResetWristEncoder();
}

// Called repeatedly when this Command is scheduled to run
void command_ResetWrist::Execute() {}

// Called once the command ends or is interrupted.
void command_ResetWrist::End(bool interrupted) {}

// Returns true when the command should end.
bool command_ResetWrist::IsFinished() {
  return false;
}
