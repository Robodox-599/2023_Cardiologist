// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/command_VisionPose.h"
#include "frc/smartdashboard/SmartDashboard.h"

command_VisionPose::command_VisionPose(subsystem_PoseTracker* PoseTracker): m_PoseTracker{PoseTracker} {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements({PoseTracker});
}

// Called when the command is initially scheduled.
void command_VisionPose::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void command_VisionPose::Execute() {
  // std::pair<frc::Pose3d, units::time::second_t> results = m_PoseTracker->getInitialPose();
  // frc::SmartDashboard::SmartDashboard::PutNumber("Pose_X", m_PoseTracker->getEstimatedGlobalPose(results.first).first.X().value());
  // frc::SmartDashboard::SmartDashboard::PutNumber("Pose_Y", m_PoseTracker->getEstimatedGlobalPose(results.first).first.Y().value());
  // m_PoseTracker->Periodic();
  // frc::SmartDashboard::SmartDashboard::PutNumber("ID", m_PoseTracker->getID());
}

// Called once the command ends or is interrupted.
void command_VisionPose::End(bool interrupted) {}

// Returns true when the command should end.
bool command_VisionPose::IsFinished() {
  return false;
}
