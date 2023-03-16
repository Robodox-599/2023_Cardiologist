// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/command_DriveTeleop.h"
#include "frc/smartdashboard/SmartDashboard.h"
 command_DriveTeleop::command_DriveTeleop(subsystem_DriveTrain* DriveTrain, subsystem_PoseTracker* PoseTracker,
                                          std::function<double()> xSpeed,
                                          std::function<double()> ySpeed,
                                          std::function<double()> zRotation,
                                          std::function<double()> IsOrientFront,
                                          std::function<double()> IsOrientBack,
                                          std::function<double()> LED,
                                          std::function<bool()> FieldRelative,
                                          std::function<bool()> OpenLoop): m_DriveTrain{DriveTrain}, 
                                                      m_PoseTracker{PoseTracker},
                                                      m_xSpeed{xSpeed},
                                                      m_ySpeed{ySpeed},
                                                      m_zRotation{zRotation},
                                                      m_IsOrientFront{IsOrientFront},
                                                      m_IsOrientBack{IsOrientBack},
                                                      m_LED{LED},
                                                      m_FieldRelative{FieldRelative},
                                                      m_OpenLoop{OpenLoop}
{
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements({DriveTrain});
  AddRequirements({PoseTracker});
}

// Called when the command is initially scheduled.
void command_DriveTeleop::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void command_DriveTeleop::Execute() {

  
  if(m_PoseTracker->HasAcceptableTargets()){
    m_DriveTrain->ImplementVisionPose(m_PoseTracker->getEstimatedGlobalPose());
  }

  m_DriveTrain->SetAutoOrient(m_IsOrientFront(), m_IsOrientBack(), m_zRotation());

  if(m_LED() > ControllerConstants::TriggerActivate){
    m_DriveTrain->SetPurpleLED();
  }else if(m_LED() < - ControllerConstants::TriggerActivate){
    m_DriveTrain->SetYellowLED();
  }

  
  
  m_DriveTrain -> SwerveDrive( m_DriveTrain-> SetThrottle( frc::ApplyDeadband(m_xSpeed(), ControllerConstants::Deadband) )* SwerveConstants::MaxSpeed,
                                m_DriveTrain-> SetThrottle( frc::ApplyDeadband(m_ySpeed(), ControllerConstants::Deadband) ) * SwerveConstants::MaxSpeed,
                                m_DriveTrain-> SetThrottle( frc::ApplyDeadband(m_zRotation(), ControllerConstants::Deadband) ) * SwerveConstants::MaxAngularVelocity,
                                m_FieldRelative(),
                                m_OpenLoop());



  
  

}

// Called once the command ends or is interrupted.
void command_DriveTeleop::End(bool interrupted) {}

// Returns true when the command should end.
bool command_DriveTeleop::IsFinished() {
  return false;
}
