// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/subsystem_PoseTracker.h"
#include "frc/smartdashboard/SmartDashboard.h"

subsystem_PoseTracker::subsystem_PoseTracker()
{
  cameras.push_back(std::make_pair(cameraOne, robotToCam));
}

std::pair<frc::Pose3d, units::time::second_t> subsystem_PoseTracker::getInitialPose()
{
  return estimator.Update();
}

int subsystem_PoseTracker::getID()
{
  return targetID;
}

bool subsystem_PoseTracker::hasTarget(){
  frc::SmartDashboard::SmartDashboard::PutNumber("Size", (int)tags.size());
  // result = camera.GetLatestResult();
  return result.HasTargets() && target.GetPoseAmbiguity() <= 0.2 && targetID >= 0 /* && targetID < (int)tags.size()*/;
}

std::pair<frc::Pose2d, units::millisecond_t> subsystem_PoseTracker::getEstimatedGlobalPose()
{
  result = camera.GetLatestResult();
  units::second_t timeStamp{result.GetTimestamp()};
  target = result.GetBestTarget();
  targetID = target.GetFiducialId();   
  auto targetPose = aprilTags.get()->GetTagPose(targetID);
  frc::Transform3d camToTarget = target.GetBestCameraToTarget();
  frc::Pose3d camPose = targetPose.value().TransformBy(camToTarget.Inverse()); 
  frc::Pose3d visionMeasurement = camPose.TransformBy(robotToCam.Inverse());
  return std::make_pair<>(visionMeasurement.ToPose2d(), timeStamp);
  // return std::make_pair(frc::Pose2d(), 0_ms);
}
// This method will be called once per scheduler run
void subsystem_PoseTracker::Periodic()
{
  
  estimator.Update();
  result = camera.GetLatestResult();
  target = result.GetBestTarget();
  hasTargets = result.HasTargets();
  yaw = target.GetYaw();
  pitch = target.GetPitch();
  area = target.GetArea();
  targetID = target.GetFiducialId();
  poseAmbiguity = target.GetPoseAmbiguity();
  
  auto targetPose = aprilTags.get()->GetTagPose(targetID);
  
  frc::Transform3d camToTarget = target.GetBestCameraToTarget();


  if(targetPose.has_value()){
    frc::Pose3d camPose = targetPose.value().TransformBy(camToTarget.Inverse()); 
    frc::Pose3d visionMeasurement = camPose.TransformBy(robotToCam.Inverse());
    frc::SmartDashboard::SmartDashboard::PutNumber("Pose_X", visionMeasurement.X().value());
  frc::SmartDashboard::SmartDashboard::PutNumber("Pose_Y", visionMeasurement.Y().value());
  frc::SmartDashboard::SmartDashboard::PutNumber("_ID_", targetID);
  frc::SmartDashboard::SmartDashboard::PutNumber("Yaw", yaw);
  frc::SmartDashboard::SmartDashboard::PutNumber("Pitch", pitch);
  frc::SmartDashboard::SmartDashboard::PutNumber("Area", area);
  frc::SmartDashboard::SmartDashboard::PutBoolean("Targets", hasTargets);
  frc::SmartDashboard::SmartDashboard::PutNumber("Ambiguity", poseAmbiguity);
  
  } 
  
  
  
  // frc::SmartDashboard::SmartDashboard::PutNumber("Pose_X_2D", estimator.Update().first.ToPose2d().X().value());
  // frc::SmartDashboard::SmartDashboard::PutNumber("Pose_Y_2D", estimator.Update().first.ToPose2d().Y().value());
  
}
