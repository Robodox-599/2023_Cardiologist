// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/command_IntakeRun.h"

command_IntakeRun::command_IntakeRun(subsystem_Intake* intake, std::function<double()> outputPower) : m_Intake{intake}, m_OutputPower{outputPower} {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements({m_Intake});
}

// Called when the command is initially scheduled.
void command_IntakeRun::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void command_IntakeRun::Execute() {
  if(m_Intake->IsIntakeOpen()) {
    m_Intake->SetIntakeWheelsOn(m_OutputPower());
    printf("Setting intake wheels on");
  }
}

// Called once the command ends or is interrupted.
void command_IntakeRun::End(bool interrupted) {}

// Returns true when the command should end.
bool command_IntakeRun::IsFinished() {
  return false;
}
