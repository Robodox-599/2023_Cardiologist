// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/command_DriveTeleop.h"
#include "frc/smartdashboard/SmartDashboard.h"
 command_DriveTeleop::command_DriveTeleop(subsystem_DriveTrain* DriveTrain,
                                                      std::function<double()> xSpeed,
                                                      std::function<double()> ySpeed,
                                                      std::function<double()> zRotation,
                                                      std::function<bool()> FieldRelative,
                                                      std::function<bool()> OpenLoop): m_DriveTrain{DriveTrain},
                                                                                          m_xSpeed{xSpeed},
                                                                                          m_ySpeed{ySpeed},
                                                                                          m_zRotation{zRotation},
                                                                                          m_FieldRelative{FieldRelative},
                                                                                          m_OpenLoop{OpenLoop}
{
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements({m_DriveTrain});
}

// Called when the command is initially scheduled.
void command_DriveTeleop::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void command_DriveTeleop::Execute() {
  frc::SmartDashboard::SmartDashboard::PutNumber("x_Speed", m_xSpeed());
  frc::SmartDashboard::SmartDashboard::PutNumber("y_Speed", m_ySpeed());
  frc::SmartDashboard::SmartDashboard::PutNumber("z_rotation", m_zRotation());
  m_DriveTrain -> SwerveDrive( m_DriveTrain-> SetThrottle(-frc::ApplyDeadband(m_xSpeed(), 0.1))* SwerveConstants::MaxSpeed,
                                m_DriveTrain-> SetThrottle(frc::ApplyDeadband(m_ySpeed(), 0.1)) * SwerveConstants::MaxSpeed,
                                m_DriveTrain-> SetThrottle(frc::ApplyDeadband(m_zRotation(), 0.1)) * SwerveConstants::MaxAngularVelocity,
                                m_FieldRelative(),
                                m_OpenLoop());

  
  
}

// Called once the command ends or is interrupted.
void command_DriveTeleop::End(bool interrupted) {}

// Returns true when the command should end.
bool command_DriveTeleop::IsFinished() {
  return false;
}
