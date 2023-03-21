// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/command_ControllerVibrate.h"

command_ControllerVibrate::command_ControllerVibrate(frc::XboxController *Drive, subsystem_Intake *Intake, frc::Timer *Timer) :
m_Driver{Drive},
m_Intake{Intake},
m_Timer{Timer}
{
  AddRequirements({m_Intake});
}

// Called when the command is initially scheduled.
void command_ControllerVibrate::Initialize() {
  if (!m_Intake->IsIntakeOpen() && m_Timer->Get() > IntakeConstants::VibrationTimeout 
              && m_Intake->GetCurrentState() != IntakeConstants::State::Empty){
    m_Driver->SetRumble(frc::GenericHID::RumbleType::kBothRumble, 0.5);
    m_Timer->Reset();
  } else{
    m_Driver->SetRumble(frc::GenericHID::RumbleType::kBothRumble, 0.0);
    m_Timer->Reset();
  }
}

// Called repeatedly when this Command is scheduled to run
void command_ControllerVibrate::Execute() {}

// Called once the command ends or is interrupted.
void command_ControllerVibrate::End(bool interrupted) {}

// Returns true when the command should end.
bool command_ControllerVibrate::IsFinished() {
  return false;
}
