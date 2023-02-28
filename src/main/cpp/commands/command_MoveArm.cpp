// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/command_MoveArm.h"

command_MoveArm::command_MoveArm(subsystem_Arm *arm, std::function<double()> X, std::function<double()> Y) : m_arm{arm},
                                                                                                             m_X{X},
                                                                                                             m_Y{Y}
{
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements({m_arm});
}

// Called when the command is initially scheduled.
void command_MoveArm::Initialize() {
  // m_arm->UnlockArm();
  // printf("ARM MOVING");
  // m_arm->MoveArm(m_X(), m_Y());
  m_arm->RunBottomArmTest();
}

// Called repeatedly when this Command is scheduled to run
void command_MoveArm::Execute() {}

// Called once the command ends or is interrupted.
void command_MoveArm::End(bool interrupted) {
  // if(m_arm->IsAtDesiredPosition()){
  //   m_arm->LockArm();
  // }
}

// Returns true when the command should end.
bool command_MoveArm::IsFinished() {
  // return m_arm->IsAtDesiredPosition();
  return true;
}
