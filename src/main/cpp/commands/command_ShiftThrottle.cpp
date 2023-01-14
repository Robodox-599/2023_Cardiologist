// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/command_ShiftThrottle.h"

command_ShiftThrottle::command_ShiftThrottle(subsystem_DriveTrain* DriveTrain): m_DriveTrain{DriveTrain} {
  // Use addRequirements() here to declare subsystem dependencies.
}

// Called when the command is initially scheduled.
void command_ShiftThrottle::Initialize() {
  m_DriveTrain -> ChangeThrottle();
}

// Called repeatedly when this Command is scheduled to run
void command_ShiftThrottle::Execute() {}

// Called once the command ends or is interrupted.
void command_ShiftThrottle::End(bool interrupted) {}

// Returns true when the command should end.
bool command_ShiftThrottle::IsFinished() {
  return true;
}
