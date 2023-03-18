// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/command_SetLED.h"
#include "frc/smartdashboard/SmartDashboard.h"

command_SetLED::command_SetLED(subsystem_LED* LED, subsystem_Intake* Intake, std::function<double()> WantsCube): 
                                                                                                  m_LED{LED}, 
                                                                                                  m_Intake{Intake},
                                                                                                  m_WantsCube{WantsCube}
{
  // Use addRequirements() here to declare subsystem dependencies.
}

// Called when the command is initially scheduled.
void command_SetLED::Initialize() {

}

// Called repeatedly when this Command is scheduled to run
void command_SetLED::Execute() {
  if(m_Intake->GetCurrentState() != IntakeConstants::State::Empty){
    m_LED->SetIntakedLED();
  }else{
    if(m_WantsCube() > ControllerConstants::TriggerActivate){
      m_LED->SetPurpleLED();
    }else if(m_WantsCube() < - ControllerConstants::TriggerActivate){
      m_LED->SetYellowLED();
    }
  }
}

// Called once the command ends or is interrupted.
void command_SetLED::End(bool interrupted) {}

// Returns true when the command should end.
bool command_SetLED::IsFinished() {
  return false;
}
