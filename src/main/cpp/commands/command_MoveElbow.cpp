// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/command_MoveElbow.h"

command_MoveElbow::command_MoveElbow(subsystem_Arm *Arm,  std::function<double()> EncPosition, 
                                                      std::function<bool()> IsWait) :m_Arm{Arm},
                                                                                    m_EncPosition{EncPosition},
                                                                                    m_IsWait{IsWait}
{
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements({m_Arm});
}

// Called when the command is initially scheduled.
void command_MoveElbow::Initialize() {
  // m_arm->UnlockArm();
  // m_arm->MoveArm(m_X(), m_Y());
}

// Called repeatedly when this Command is scheduled to run
void command_MoveElbow::Execute() {
  // m_arm->RunArmTest(m_X(), m_Y(), m_Tilt());
}

// Called once the command ends or is interrupted.
void command_MoveElbow::End(bool interrupted) {
  // if(m_arm->IsAtDesiredPosition()){
  //   m_arm->LockArm();
  // }
}

// Returns true when the command should end.
bool command_MoveElbow::IsFinished() {
  // return m_arm->IsAtDesiredPosition();
  return false;
}
