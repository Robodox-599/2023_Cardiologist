// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/command_DriveAuton.h"
#include "frc/smartdashboard/SmartDashboard.h"

command_DriveAuton::command_DriveAuton(subsystem_DriveTrain* DriveTrain, subsystem_PoseTracker* PoseTracker, std::string TrajFilePath, frc::DriverStation::Alliance AllianceColor, bool ToReset):
m_DriveTrain{DriveTrain}, m_PoseTracker{PoseTracker}, m_ToReset{ToReset}, m_DriveController{AutoConstants::XPID, AutoConstants::YPID, AutoConstants::ThetaPID} {
  // pathplanner::PPHolonomicDriveController m_DriveController{AutoConstants::XPID, AutoConstants::YPID, AutoConstants::ZPID};
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements({m_DriveTrain});
  // m_Trajectory = frc::TrajectoryUtil::FromPathweaverJson(deployDirectory.string());
  m_Trajectory = pathplanner::PathPlanner::loadPath(TrajFilePath, pathplanner::PathConstraints(AutoConstants::MaxSpeed, AutoConstants::MaxAccel) );  
  pathplanner::PathPlannerTrajectory::transformTrajectoryForAlliance(m_Trajectory, AllianceColor);
  }


// Called when the command is initially scheduled.
void command_DriveAuton::Initialize() {
  m_Timer.Stop();
  m_Timer.Reset();
  m_Timer.Start();

  if( m_ToReset ){
    m_DriveTrain->ResetOdometry(m_DriveTrain->GetYaw(), {m_Trajectory.getInitialState().pose.Translation(), m_Trajectory.getInitialState().holonomicRotation});
  }
}

// Called repeatedly when this Command is scheduled to run
void command_DriveAuton::Execute() {
  if(m_PoseTracker->hasTarget()){
    m_DriveTrain->ImplementVisionPose(m_PoseTracker->getEstimatedGlobalPose());    
  }
  pathplanner::PathPlannerTrajectory::PathPlannerState state = m_Trajectory.sample(m_Timer.Get()); 
  auto chassisSpeeds = m_DriveController.Calculate(m_DriveTrain->GetPose(), 
                                                  state.pose,
                                                  state.velocity,
                                                  state.holonomicRotation);
  // chassisSpeeds.vy = units::meters_per_second_t{ fabs(chassisSpeeds.vy.value()) };
  auto ModuleStates = SwerveConstants::m_kinematics.ToSwerveModuleStates(chassisSpeeds);
  SwerveConstants::m_kinematics.DesaturateWheelSpeeds(&ModuleStates, AutoConstants::MaxSpeed);
  m_DriveTrain->SetModuleStates(ModuleStates);
  
  frc::SmartDashboard::SmartDashboard::PutNumber("Velocity_X: ", chassisSpeeds.vx.value());
  frc::SmartDashboard::SmartDashboard::PutNumber("Velocity_Y: ", chassisSpeeds.vy.value());
  frc::SmartDashboard::SmartDashboard::PutNumber("Pose_x: ", m_DriveTrain->GetPose().X().value());
  frc::SmartDashboard::SmartDashboard::PutNumber("Pose_Y: ", m_DriveTrain->GetPose().Y().value());
  // frc::SmartDashboard::SmartDashboard::PutNumber("Angular Vel (Real): ", m_DriveTrain->);
  frc::SmartDashboard::SmartDashboard::PutNumber("Angular Vel: ", state.angularVelocity.value());
  frc::SmartDashboard::SmartDashboard::PutNumber("Acceleration: ", state.acceleration.value());
  frc::SmartDashboard::SmartDashboard::PutNumber("Holonomic Rotation: ", state.holonomicRotation.Degrees().value());
  frc::SmartDashboard::SmartDashboard::PutNumber("chassis angular velocity: ", chassisSpeeds.omega());
}

// Called once the command ends or is interrupted.
void command_DriveAuton::End(bool interrupted) {
  m_DriveTrain -> SwerveDrive(0_mps, 0_mps, 0_rad_per_s, false, false);
}

// Returns true when the command should end.
bool command_DriveAuton::IsFinished() {
  return m_Timer.Get() >= (m_Trajectory.getTotalTime());
}
