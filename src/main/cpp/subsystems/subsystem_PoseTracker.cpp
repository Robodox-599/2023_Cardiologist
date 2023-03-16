// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/subsystem_PoseTracker.h"
#include "frc/smartdashboard/SmartDashboard.h"

subsystem_PoseTracker::subsystem_PoseTracker()
{
  cameras.push_back(std::make_pair(cameraOne, robotToCam));

  SetAllianceOrigin(m_Alliance);
  m_Chooser.SetDefaultOption("BLUE_ALLIANCE", BLUE_ALLIANCE);
  m_Chooser.AddOption("RED_ALLIANCE", RED_ALLIANCE);
  frc::SmartDashboard::PutData(&m_Chooser);

}



std::pair<frc::Pose3d, units::time::second_t> subsystem_PoseTracker::getInitialPose()
{

  return estimator.Update();

}

int subsystem_PoseTracker::getID()
{

  return targetID;

}

bool subsystem_PoseTracker::HasAcceptableTargets(){
  // frc::SmartDashboard::SmartDashboard::PutNumber("Size", (int)tags.size());
  // result = camera.GetLatestResult();
  return result.HasTargets() && (target.GetPoseAmbiguity() <= 0.1) && ( targetID > 0  && targetID < (int)tags.size() );
}

frc::DriverStation::Alliance subsystem_PoseTracker::GetAlliance(){
  return m_Alliance;
} 

void subsystem_PoseTracker::SetAllianceOrigin(frc::DriverStation::Alliance AllianceColor){
  if(m_Alliance == frc::DriverStation::Alliance::kRed){
    aprilTags.get()->SetOrigin(frc::AprilTagFieldLayout::OriginPosition::kRedAllianceWallRightSide);
  }else if(m_Alliance == frc::DriverStation::Alliance::kBlue){
    aprilTags.get()->SetOrigin(frc::AprilTagFieldLayout::OriginPosition::kBlueAllianceWallRightSide);
  }
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

  if(m_Chooser.GetSelected() != m_Alliance){
    m_Alliance = m_Chooser.GetSelected();
    SetAllianceOrigin(m_Alliance);
  } 
  
   frc::SmartDashboard::SmartDashboard::PutNumber("Pose_X_2D", estimator.Update().first.ToPose2d().X().value());
   frc::SmartDashboard::SmartDashboard::PutNumber("Pose_Y_2D", estimator.Update().first.ToPose2d().Y().value());
  
}
