// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/command_IntakeCube.h"

command_IntakeCube::command_IntakeCube(subsystem_Intake* intake) : m_Intake{intake} {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements({m_Intake});
}

// Called when the command is initially scheduled.
void command_IntakeCube::Initialize() {
  m_Intake->IntakeOpen();
  m_Intake->SetIntake();
}

// Called repeatedly when this Command is scheduled to run
void command_IntakeCube::Execute() {
  printf("Setting Intake wheels on");
}

// Called once the command ends or is interrupted.
void command_IntakeCube::End(bool interrupted) {
  m_Intake->IntakeClose();
  m_Intake->SetPassive();
}

// Returns true when the command should end.
bool command_IntakeCube::IsFinished() {
  return m_Intake->GetCurrent() > IntakeConstants::CurrentLimit;
}