// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/command_MoveWrist.h"

command_MoveWrist::command_MoveWrist(subsystem_Arm *Arm, std::function<double()> EncPosition, 
                                                      std::function<bool()> IsWait) :m_Arm{Arm},
                                                                                    m_EncPosition{EncPosition},
                                                                                    m_IsWait{IsWait}
{
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements({m_Arm});
}

// Called when the command is initially scheduled.
void command_MoveWrist::Initialize() {
  m_Timer.Start();
  // m_arm->UnlockArm();
  // m_arm->MoveArm(m_X(), m_Y());
  m_Arm->SetWristByPosition(m_EncPosition());
}

// Called repeatedly when this Command is scheduled to run
void command_MoveWrist::Execute() {
  if(!m_Arm->IsWristAtDesiredPosition()){
    m_Timer.Reset();
  }
      // frc::SmartDashboard::PutNumber("ElbowPosition", DesiredElbowPosition);
    // frc::SmartDashboard::PutNumber("ShoulderPosition", DesiredShoulderPosition);
    //  frc::SmartDashboard::PutNumber("WristPosition", m_EncPosition());
}

// Called once the command ends or is interrupted.
void command_MoveWrist::End(bool interrupted) {
  // if(m_arm->IsAtDesiredPosition()){
  //   m_arm->LockArm();
  // }

}

// Returns true when the command should end.
bool command_MoveWrist::IsFinished() {
  // return m_arm->IsAtDesiredPosition();
  if( m_IsWait() && m_Timer.Get() < ArmConstants::ManualTimer){
    return false;
  }else{
    return true;
  }
  // if(m_IsWait()){
  //  return m_Arm->IsWristAtDesiredPosition();
  // }
  // else if(!m_IsWait()){
  //   return true;
  // }
}

