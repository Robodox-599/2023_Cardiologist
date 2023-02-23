// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/command_MoveArmManually.h"

command_MoveArmManually::command_MoveArmManually(subsystem_Arm *arm, std::function<double()> leftJoystick, std::function<double()> rightJoystick) : 
m_arm{arm},
m_LeftJoystickInput{leftJoystick},
m_RightJoystickInput{rightJoystick}
{
  AddRequirements({m_arm});
}

// Called when the command is initially scheduled.
void command_MoveArmManually::Initialize() {
  m_Timer.Reset();
  m_Timer.Start();
  m_arm->UnlockArm();
  m_arm->MoveArmManually(m_LeftJoystickInput(), m_RightJoystickInput());
}

// Called repeatedly when this Command is scheduled to run
void command_MoveArmManually::Execute() {}

// Called once the command ends or is interrupted.
void command_MoveArmManually::End(bool interrupted) {
  m_arm->LockArm();
}

// Returns true when the command should end.
bool command_MoveArmManually::IsFinished() {
  return (m_Timer.Get() >= ArmConstants::ManualTimer);
}
