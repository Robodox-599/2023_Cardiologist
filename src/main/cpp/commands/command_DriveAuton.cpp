// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/command_DriveAuton.h"

command_DriveAuton::command_DriveAuton(subsystem_DriveTrain* DriveTrain, std::string TrajFilePath, bool ToReset):
m_DriveTrain{DriveTrain}, m_ToReset{ToReset} {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements({m_DriveTrain});
  fs::path deployDirectory = frc::filesystem::GetDeployDirectory();
  deployDirectory = deployDirectory / "pathplanner" / TrajFilePath;
  m_Trajectory = frc::TrajectoryUtil::FromPathweaverJson(deployDirectory.string());

  
  
  
  
}



// Called when the command is initially scheduled.
void command_DriveAuton::Initialize() {
  m_Timer.Stop();
  m_Timer.Reset();
  m_Timer.Start();

  if( m_ToReset ){
    m_DriveTrain->ResetOdometry(m_Trajectory.InitialPose());
  }
  frc::ProfiledPIDController<units::radians> ThetaController{AutoConstants::AutoDriveKP, 
                                              AutoConstants::AutoDriveKI, 
                                              AutoConstants::AutoDriveKD, 
                                              frc::TrapezoidProfile<units::radians>::Constraints{AutoConstants::MaxAngularSpeed,
                                                                                                 AutoConstants::MaxAngularAccel}};

  ThetaController.EnableContinuousInput(-units::radian_t{3.14}, units::radian_t{ 3.14} );

  frc2::SwerveControllerCommand<4> swerveControllerCommand(m_Trajectory,
                                                            [this]{return m_DriveTrain->GetPose();}, 
                                                            SwerveConstants::m_kinematics,
                                                            frc2::PIDController{10,10,10},
                                                            frc2::PIDController{10,10,10},
                                                            ThetaController,
                                                            [this](auto moduleStates) { m_DriveTrain->SetModuleStates(moduleStates); },
                                                            {m_DriveTrain});
                                                            
                                                            
                                                            //.AndThen([]{ return m_DriveTrain->DriveTrain(0.0, 0.0, 0.0, false, false)});
                                                            
                                                            
  


}

// Called repeatedly when this Command is scheduled to run
void command_DriveAuton::Execute() {}

// Called once the command ends or is interrupted.
void command_DriveAuton::End(bool interrupted) {
  m_DriveTrain -> SwerveDrive(0_mps, 0_mps, 0_rad_per_s, false, false);
}

// Returns true when the command should end.
bool command_DriveAuton::IsFinished() {
  return m_Timer.Get() >= (m_Trajectory.TotalTime() *1.2);
}
