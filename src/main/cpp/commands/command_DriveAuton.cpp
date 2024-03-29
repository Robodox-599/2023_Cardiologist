// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/command_DriveAuton.h"
#include "frc/smartdashboard/SmartDashboard.h"

command_DriveAuton::command_DriveAuton(subsystem_DriveTrain* DriveTrain, subsystem_PoseTracker* PoseTracker, std::string TrajFilePath, bool ToReset):
m_DriveTrain{DriveTrain}, m_PoseTracker{PoseTracker}, m_ToReset{ToReset},  m_DriveController{AutoConstants::XPID, AutoConstants::YPID, AutoConstants::ThetaPID} {
  // pathplanner::PPHolonomicDriveController m_DriveController{AutoConstants::XPID, AutoConstants::YPID, AutoConstants::ZPID};
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements({m_DriveTrain});
  AddRequirements({m_PoseTracker});
  // m_Trajectory = frc::TrajectoryUtil::FromPathweaverJson(deployDirectory.string());

  if(TrajFilePath.starts_with("Yes^3") ){
    m_Trajectory = pathplanner::PathPlanner::loadPath(TrajFilePath, pathplanner::PathConstraints(4_mps, units::meters_per_second_squared_t{4}) );  

  }else{
    m_Trajectory = pathplanner::PathPlanner::loadPath(TrajFilePath, pathplanner::PathConstraints(AutoConstants::MaxSpeed, AutoConstants::MaxAccel) );  

  }






// command_DriveAuton::command_DriveAuton(subsystem_DriveTrain* DriveTrain, subsystem_PoseTracker* PoseTracker, std::string TrajFilePath, bool ToReset):
// m_DriveTrain{DriveTrain}, m_PoseTracker{PoseTracker}, m_ToReset{ToReset}, m_IsCustomMotion{false}, m_DriveController{AutoConstants::XPID, AutoConstants::YPID, AutoConstants::ThetaPID} {
  // pathplanner::PPHolonomicDriveController m_DriveController{AutoConstants::XPID, AutoConstants::YPID, AutoConstants::ZPID};
  // Use addRequirements() here to declare subsystem dependencies.
  // AddRequirements({m_DriveTrain});
  // AddRequirements({m_PoseTracker});
  // m_Trajectory = frc::TrajectoryUtil::FromPathweaverJson(deployDirectory.string());


  // m_Trajectory = pathplanner::PathPlanner::loadPath(TrajFilePath, pathplanner::PathConstraints(AutoConstants::MaxSpeed, AutoConstants::MaxAccel) );  
  
  
}

// Called when the command is initially scheduled.
void command_DriveAuton::Initialize() {
  m_Trajectory = pathplanner::PathPlannerTrajectory::transformTrajectoryForAlliance(m_Trajectory, frc::DriverStation::GetAlliance());
  
  m_Timer.Stop();
  m_Timer.Reset();
  m_Timer.Start();

  if( m_ToReset ){
    m_DriveTrain->ResetOdometry(m_Trajectory.getInitialHolonomicPose());
  }
  frc::SmartDashboard::GetNumber("InitialX", m_Trajectory.getInitialHolonomicPose().X().value());
  frc::SmartDashboard::GetNumber("InitialY", m_Trajectory.getInitialHolonomicPose().Y().value());
  frc::SmartDashboard::GetNumber("InitialHeading", m_Trajectory.getInitialHolonomicPose().Rotation().Degrees().value());

  
}

// Called repeatedly when this Command is scheduled to run
void command_DriveAuton::Execute() {
  // if(m_DriveTrain->HasAcceptableTargets()){
  //   m_DriveTrain->ImplementVisionPose(m_DriveTrain->getEstimatedGlobalPose());    
  // }
  // m_DriveTrain->ImplementVisionPose(m_PoseTracker->GetEstimatedGlobalPose());

  pathplanner::PathPlannerTrajectory::PathPlannerState state = m_Trajectory.sample(m_Timer.Get()); 
  auto chassisSpeeds = m_DriveController.Calculate(m_DriveTrain->GetPose(), 
                                                  state.pose,
                                                  state.velocity,
                                                  state.holonomicRotation);
  // chassisSpeeds.vy = units::meters_per_second_t{ fabs(chassisSpeeds.vy.value()) };
  auto ModuleStates = SwerveConstants::m_kinematics.ToSwerveModuleStates(chassisSpeeds);
  SwerveConstants::m_kinematics.DesaturateWheelSpeeds(&ModuleStates, AutoConstants::MaxSpeed);
  m_DriveTrain->SetModuleStates(ModuleStates);

  
}

// Called once the command ends or is interrupted.
void command_DriveAuton::End(bool interrupted) {
  m_DriveTrain -> SwerveDrive(0_mps, 0_mps, 0_rad_per_s, false, false);
}

// Returns true when the command should end.
bool command_DriveAuton::IsFinished() {
  return m_Timer.Get() >= (m_Trajectory.getTotalTime());
}
