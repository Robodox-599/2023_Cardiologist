// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/command_TimeOut.h"

command_TimeOut::command_TimeOut(std::function<double()> Time):m_Timer{}, m_Time{Time} {
  // Use addRequirements() here to declare subsystem dependencies.

}

// Called when the command is initially scheduled.
void command_TimeOut::Initialize() {
  m_Timer.Reset();
  m_Timer.Start();

}

// Called repeatedly when this Command is scheduled to run
void command_TimeOut::Execute() {}

// Called once the command ends or is interrupted.
void command_TimeOut::End(bool interrupted) {}

// Returns true when the command should end.
bool command_TimeOut::IsFinished() {
  return m_Timer.Get() > units::second_t{m_Time()};
}
