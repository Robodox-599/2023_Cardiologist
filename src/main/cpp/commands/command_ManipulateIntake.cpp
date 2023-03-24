// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/command_ManipulateIntake.h"

command_ManipulateIntake::command_ManipulateIntake(subsystem_Intake* Intake, subsystem_LED* LED, 
                                                          frc::XboxController* DriveController, std::function<double()> WantsCube,
                                                          std::function<bool()> IsIntake, std::function<bool()> IsOutake): m_Intake{Intake}, 
                                                                                                                                    m_LED{LED}, 
                                                                                                                                    m_DriveController{DriveController},
                                                                                                                                    m_WantsCube{WantsCube},
                                                                                                                                    m_IsIntake{IsIntake},
                                                                                                                                    m_IsOutake{IsOutake} {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements({m_Intake});
  AddRequirements({m_LED});

}

// Called when the command is initially scheduled.
void command_ManipulateIntake::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void command_ManipulateIntake::Execute() {
  if(m_Intake->GetCurrentState() != IntakeConstants::State::Empty){
    m_LED->SetIntakedLED();
  }else if(m_WantsCube() > ControllerConstants::TriggerActivate){
    m_LED->SetPurpleLED();
  }else if(m_WantsCube() < - ControllerConstants::TriggerActivate){
    m_LED->SetYellowLED();
  }

  if(m_IsIntake()){
    m_Intake->IntakeOpen();

  }else if(m_IsOutake()){
    m_Intake->IntakeClose();
  }


}

// Called once the command ends or is interrupted.
void command_ManipulateIntake::End(bool interrupted) {}

// Returns true when the command should end.
bool command_ManipulateIntake::IsFinished() {
  return false;
}
