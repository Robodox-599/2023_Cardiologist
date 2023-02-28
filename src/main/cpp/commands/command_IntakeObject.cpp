// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/command_IntakeObject.h"

command_IntakeObject::command_IntakeObject(subsystem_Intake* intake) : m_Intake{intake} {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements({m_Intake});
}

// Called when the command is initially scheduled.
void command_IntakeObject::Initialize() {
  m_Intake->IntakeOpen();
}

// Called repeatedly when this Command is scheduled to run
void command_IntakeObject::Execute() {
  m_Intake->SetIntakeWheelsOn(true);
  printf("Setting Intake wheels on");
}

// Called once the command ends or is interrupted.
void command_IntakeObject::End(bool interrupted) {
  m_Intake->IntakeClose();
  m_Intake->SetIntakeWheelsOff();
}

// Returns true when the command should end.
bool command_IntakeObject::IsFinished() {
  return (m_Intake->GetCurrentState() != "Empty");
}