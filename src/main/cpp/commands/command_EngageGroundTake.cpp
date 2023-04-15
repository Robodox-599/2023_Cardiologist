// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/command_EngageGroundTake.h"

command_EngageGroundTake::command_EngageGroundTake(subsystem_GroundTake* GroundTake): m_GroundTake{GroundTake}, m_Timer{} {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements({m_GroundTake});
}

// Called when the command is initially scheduled.
void command_EngageGroundTake::Initialize() {
  m_Timer.Restart();
  if(! m_GroundTake->IsCubeDetected()){
      m_GroundTake->ExtendGroundTake();
      m_GroundTake->RunIntake();
  }
}

// Called repeatedly when this Command is scheduled to run
void command_EngageGroundTake::Execute() {
  if(! m_GroundTake->IsCubeDetected()){
    m_Timer.Restart();
  }
}

// Called once the command ends or is interrupted.
void command_EngageGroundTake::End(bool interrupted) {
  m_GroundTake->RetractGroundTake();
  m_GroundTake->StopIntake();
}

// Returns true when the command should end.
bool command_EngageGroundTake::IsFinished() {
  // return m_Timer.Get() > 0.05_s;
  return m_GroundTake->IsCubeDetected();
}
