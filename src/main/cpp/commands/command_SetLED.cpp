// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/command_SetLED.h"
#include "frc/smartdashboard/SmartDashboard.h"

command_SetLED::command_SetLED(subsystem_DriveTrain* DriveTrain, std::function<bool()> WantsCube): m_DriveTrain{DriveTrain},
                                                                                                  m_WantsCube{WantsCube}
{
  // Use addRequirements() here to declare subsystem dependencies.
}

// Called when the command is initially scheduled.
void command_SetLED::Initialize() {
  frc::SmartDashboard::PutBoolean("SET_LED BOOLEAN", m_WantsCube() );
  if(m_WantsCube()){
    
    m_DriveTrain->SetPurpleLED();
  }else{
    m_DriveTrain->SetYellowLED();
  }
}

// Called repeatedly when this Command is scheduled to run
void command_SetLED::Execute() {}

// Called once the command ends or is interrupted.
void command_SetLED::End(bool interrupted) {}

// Returns true when the command should end.
bool command_SetLED::IsFinished() {
  return true;
}
