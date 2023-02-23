// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/command_MoveArmManually.h"

command_MoveArmManually::command_MoveArmManually(subsystem_Arm *arm, std::function<double()> leftJoystick, std::function<double()> rightJoystick) : m_arm{arm},
                                                                                                                                                    m_LeftJoystickInput{leftJoystick},
                                                                                                                                                    m_RightJoystickInput{rightJoystick}
{
  AddRequirements({m_arm});
}

// Called when the command is initially scheduled.
void command_MoveArmManually::Initialize()
{
  m_Timer.Start();
}
// Called repeatedly when this Command is scheduled to run
void command_MoveArmManually::Execute()
{
  if (fabs(m_LeftJoystickInput()) > ControllerConstants::Deadband || fabs(m_RightJoystickInput()) > ControllerConstants::Deadband)
  {
    m_arm->UnlockArm();
    m_arm->MoveArmManually(m_LeftJoystickInput(), m_RightJoystickInput());
    m_Timer.Reset();
  }
  if (fabs(m_LeftJoystickInput()) < ControllerConstants::Deadband && fabs(m_RightJoystickInput()) < ControllerConstants::Deadband && m_Timer.Get() >= ArmConstants::ManualTimer)
  {
    m_arm->LockArm();
    m_Timer.Reset();
  }
}

// Called once the command ends or is interrupted.
void command_MoveArmManually::End(bool interrupted)
{
}

// Returns true when the command should end.
bool command_MoveArmManually::IsFinished()
{
  return false;
}
