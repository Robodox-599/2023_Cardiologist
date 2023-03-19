// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/command_Balance.h"

command_Balance::command_Balance(subsystem_DriveTrain* DriveTrain):m_DriveTrain{DriveTrain}, m_Timer{}  {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements({m_DriveTrain});
}

// Called when the command is initially scheduled.
void command_Balance::Initialize() {
  m_DriveTrain->EnableBalanceCorrection();
  m_Timer.Reset();
  m_Timer.Start();
}

// Called repeatedly when this Command is scheduled to run
void command_Balance::Execute() {
  if(!m_DriveTrain->IsBalanced()){
    m_Timer.Reset();
  }
  m_DriveTrain->SwerveDrive(0_mps, 0_mps, 0_rad_per_s, true, false);
}

// Called once the command ends or is interrupted.
void command_Balance::End(bool interrupted) {
  m_DriveTrain->DisableBalanceCorrection();
}

// Returns true when the command should end.
bool command_Balance::IsFinished() {
  return m_Timer.Get() > 4_s;
}
