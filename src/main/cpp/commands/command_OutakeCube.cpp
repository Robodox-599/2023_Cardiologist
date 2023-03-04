// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/command_OutakeCube.h"

command_OutakeCube::command_OutakeCube(subsystem_Intake* intake) : m_Intake{intake}, m_Timer{} {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements({intake});
}

// Called when the command is initially scheduled.
void command_OutakeCube::Initialize() {
  m_Timer.Reset();
  m_Timer.Start();
  m_Intake->SetOutake();

  


}

// Called repeatedly when this Command is scheduled to run
void command_OutakeCube::Execute() {

}

// Called once the command ends or is interrupted.
void command_OutakeCube::End(bool interrupted) {
  m_Intake->SetPassive();
}

// Returns true when the command should end.
bool command_OutakeCube::IsFinished() {
  return m_Timer.Get() > IntakeConstants::TimerConstant;
}
