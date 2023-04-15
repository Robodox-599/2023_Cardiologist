// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/command_Blink.h"

command_Blink::command_Blink(subsystem_LED* LED, std::function<int()> IntakeType): m_LED{LED}, m_Timer{}, m_IntakeType{IntakeType} {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements({m_LED});
}

// Called when the command is initially scheduled.
void command_Blink::Initialize() {
  m_Timer.Restart();

}

// Called repeatedly when this Command is scheduled to run
void command_Blink::Execute() {
  if(m_IntakeType() == 0){
    if(m_Timer.Get().value() < 0.1 || (m_Timer.Get().value() >= 0.2 && m_Timer.Get().value() < 0.3) || (m_Timer.Get().value() >= 0.4 && m_Timer.Get().value() < 0.5)){
      m_LED->SetPurpleLED();
    }
    if((m_Timer.Get().value() >= 0.1 && m_Timer.Get().value() < 0.2) || (m_Timer.Get().value() >= 0.3 && m_Timer.Get().value() < 0.4)){
      m_LED->SetOffLED();
    }
  } 
  if(m_IntakeType() == 1) {
    if(m_Timer.Get().value() < 0.1 || (m_Timer.Get().value() >= 0.2 && m_Timer.Get().value() < 0.3) || (m_Timer.Get().value() >= 0.4 && m_Timer.Get().value() < 0.5)){
      m_LED->SetPurpleLED();
    }
    if((m_Timer.Get().value() >= 0.1 && m_Timer.Get().value() < 0.2) || (m_Timer.Get().value() >= 0.3 && m_Timer.Get().value() < 0.4)){
      m_LED->SetYellowLED();
    }
  }
}

// Called once the command ends or is interrupted.
void command_Blink::End(bool interrupted) {
  m_LED->SetStandbyLED();
}

// Returns true when the command should end.
bool command_Blink::IsFinished() {
  return m_Timer.Get() >= 0.5_s;
}
