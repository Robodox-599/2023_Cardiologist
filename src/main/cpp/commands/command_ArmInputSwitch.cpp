// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/command_ArmInputSwitch.h"

command_ArmInputSwitch::command_ArmInputSwitch(subsystem_Arm *arm) : m_arm{arm} {
  // Use addRequirements() here to declare subsystem dependencies.
}

// Called when the command is initially scheduled.
void command_ArmInputSwitch::Initialize() { 
  // m_arm->ManualMacroSwitch();
  m_arm -> ResetWrist();
}

// Called repeatedly when this Command is scheduled to run
void command_ArmInputSwitch::Execute() {}

// Called once the command ends or is interrupted.
void command_ArmInputSwitch::End(bool interrupted) {}

// Returns true when the command should end.
bool command_ArmInputSwitch::IsFinished() {
  return true;
}
