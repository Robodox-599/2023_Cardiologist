// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/command_IntakeClamp.h"

command_IntakeClamp::command_IntakeClamp(subsystem_Intake* intake) : m_Intake{intake} {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements({m_Intake});
}

// Called when the command is initially scheduled.
void command_IntakeClamp::Initialize() {
  printf("(command_IntakeClamp) Clamp command started");
  if(m_Intake->IsIntakeOpen()) {
    m_Intake->IntakeClose();
    printf("(command_IntakeClamp) Intake close");
  } else {
    m_Intake->IntakeOpen();
    printf("(command_IntakeClamp) Intake open");
  }
}

// Called repeatedly when this Command is scheduled to run
void command_IntakeClamp::Execute() {}

// Called once the command ends or is interrupted.
void command_IntakeClamp::End(bool interrupted) {}

// Returns true when the command should end.
bool command_IntakeClamp::IsFinished() {
  return true;
}
