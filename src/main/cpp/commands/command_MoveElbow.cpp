// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/command_MoveElbow.h"

command_MoveElbow::command_MoveElbow(subsystem_Arm *Arm,  std::function<double()> EncPosition, 
                                                      std::function<bool()> IsWait, std::function<double()> Threshold) :m_Arm{Arm},
                                                                                    m_EncPosition{EncPosition},
                                                                                    m_IsWait{IsWait},
                                                                                    m_Threshold{Threshold}
{
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements({m_Arm});
}

// Called when the command is initially scheduled.
void command_MoveElbow::Initialize() {
  m_Timer.Start();
  // m_arm->UnlockArm();
  // m_arm->MoveArm(m_X(), m_Y());
  m_Arm->SetElbowByPosition(m_EncPosition());
}

// Called repeatedly when this Command is scheduled to run
void command_MoveElbow::Execute() {
    if(!m_Arm->IsElbowAtDesiredPosition()){
    m_Timer.Reset();
  }
  // m_arm->RunArmTest(m_X(), m_Y(), m_Tilt());
      //  frc::SmartDashboard::PutNumber("ElbowPosition", m_EncPosition());
    // frc::SmartDashboard::PutNumber("ShoulderPosition", DesiredShoulderPosition);
    // frc::SmartDashboard::PutNumber("WristPosition", DesiredWristPostion);
}

// Called once the command ends or is interrupted.
void command_MoveElbow::End(bool interrupted) {

}

// Returns true when the command should end.
bool command_MoveElbow::IsFinished() {
  // return m_arm->IsAtDesiredPosition();
  if( m_IsWait() && m_Timer.Get() < ArmConstants::ManualTimer){
    if(m_Arm->ElbowThreshold(m_Threshold())){
      return true;
    }
    return false;
  }else{
    return true;
  }
  //     if(m_IsWait()){
  //  return m_Arm->IsElbowAtDesiredPosition();
  // }
  // else if(!m_IsWait()){
  //   return true;
  // }
  }
