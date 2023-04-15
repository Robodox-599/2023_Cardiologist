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
                                          std::function<bool()> FieldRelative,
                                          std::function<bool()> OpenLoop): m_DriveTrain{DriveTrain}, m_PoseTracker{PoseTracker},
                                                      m_xSpeed{xSpeed},
                                                      m_ySpeed{ySpeed},
                                                      m_zRotation{zRotation},
                                                      m_IsOrientFront{IsOrientFront},
                                                      m_IsOrientBack{IsOrientBack},
                                                      m_FieldRelative{FieldRelative},
                                                      m_OpenLoop{OpenLoop}
{
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements({m_DriveTrain});
  AddRequirements({m_PoseTracker});
}

// Called when the command is initially scheduled.
void command_DriveTeleop::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void command_DriveTeleop::Execute() {

    // m_DriveTrain->ImplementVisionPose(m_PoseTracker->GetEstimatedGlobalPose());

    // std::pair<frc::Pose2d, units::second_t> Pose = m_PoseTracker->GetEstimatedGlobalPose().;
    // frc::SmartDashboard::PutNumber("VisionX")

  

  m_DriveTrain -> SetAutoOrient(m_IsOrientFront(), m_IsOrientBack(), m_zRotation());

  double TransformedXSpeed = 0.0;
  double TransformedYSpeed = 0.0;
  double TransformedZRot = 0.0;
  if(fabs(m_xSpeed()) > ControllerConstants::Deadband){
    TransformedXSpeed = m_DriveTrain->SetThrottle(m_xSpeed());
  }

  if(fabs(m_ySpeed()) > ControllerConstants::Deadband){
    TransformedYSpeed = m_DriveTrain->SetThrottle(m_ySpeed());
  }

  if(fabs(m_zRotation()) > ControllerConstants::Deadband){
    TransformedZRot = m_DriveTrain->SetThrottle(m_zRotation());
  }


  


  
  
  // m_DriveTrain -> SwerveDrive( m_DriveTrain -> SetThrottle( frc::ApplyDeadband(m_xSpeed(), ControllerConstants::Deadband) )* SwerveConstants::MaxSpeed,
  //                               m_DriveTrain -> SetThrottle( frc::ApplyDeadband(m_ySpeed(), ControllerConstants::Deadband) ) * SwerveConstants::MaxSpeed,
  //                               m_DriveTrain -> SetThrottle( frc::ApplyDeadband(m_zRotation(), ControllerConstants::Deadband) ) * SwerveConstants::MaxAngularVelocity,
  //                               m_FieldRelative(),
  //                               m_OpenLoop());
  m_DriveTrain -> SwerveDrive( TransformedXSpeed *  SwerveConstants::MaxSpeed,
                                TransformedYSpeed * SwerveConstants::MaxSpeed,
                                TransformedZRot * SwerveConstants::MaxAngularVelocity,
                                m_FieldRelative(),
                                m_OpenLoop());



  
  

}

// Called once the command ends or is interrupted.
void command_DriveTeleop::End(bool interrupted) {}

// Returns true when the command should end.
bool command_DriveTeleop::IsFinished() {
  return false;
}
