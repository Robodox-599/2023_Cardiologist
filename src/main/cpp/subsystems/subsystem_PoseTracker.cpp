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
  frc::SmartDashboard::PutNumber("ID DETECTED", 0);

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
  // frc::SmartDashboard::PutBoolean("resultTargets", result.HasTargets());
  if(result.HasTargets()){


    target = result.GetBestTarget();
    targetID = target.GetFiducialId();   
    frc::Transform3d camToTarget = target.GetBestCameraToTarget();

    return  (target.GetPoseAmbiguity() <= 0.01) &&  ( target.GetFiducialId() > 0  && target.GetFiducialId() <= (int)tags.size() );

  } else{
    return false;
  }
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



std::pair<frc::Pose2d, units::second_t> subsystem_PoseTracker::getEstimatedGlobalPose()
{
  
  result = camera.GetLatestResult();
  units::second_t timeStamp{result.GetTimestamp()};
  target = result.GetBestTarget();
  targetID = target.GetFiducialId();   
  auto targetPose = aprilTags.get()->GetTagPose(targetID);
  frc::Transform3d camToTarget = target.GetBestCameraToTarget();
  frc::Pose3d camPose = targetPose.value().TransformBy(camToTarget.Inverse()); 
  frc::Pose3d visionMeasurement = camPose.TransformBy(robotToCam.Inverse());
  frc::SmartDashboard::PutNumber("VisionMesX", visionMeasurement.X().value());
  frc::SmartDashboard::PutNumber("VisionMesY", visionMeasurement.Y().value());
  frc::SmartDashboard::PutNumber("PoseAmbiguity", target.GetPoseAmbiguity());
  frc::SmartDashboard::PutNumber("ToTarget", camToTarget.X().value());
  frc::SmartDashboard::PutNumber("ID DETECTED", targetID);
  // frc::SmartDashboard::PutNumber("ID DETECTED", targetID);
  return std::make_pair<>(visionMeasurement.ToPose2d(), timeStamp);
  // return std::make_pair(frc::Pose2d(), 0_ms);
}



// This method will be called once per scheduler run
void subsystem_PoseTracker::Periodic()
{
  result = camera.GetLatestResult();


  if(m_Chooser.GetSelected() != m_Alliance){
    m_Alliance = m_Chooser.GetSelected();
    SetAllianceOrigin(m_Alliance);
  } 

  frc::SmartDashboard::PutNumber("Alliance", frc::DriverStation::GetAlliance());
  frc::SmartDashboard::PutNumber("Number", frc::DriverStation::GetLocation());

  frc::SmartDashboard::PutBoolean("HasAcceptableTarget", HasAcceptableTargets());
  if(HasAcceptableTargets()){
    frc::Pose2d Pose = getEstimatedGlobalPose().first;
    // frc::SmartDashboard::PutNumber("Vision X", Pose.X().value());
    // frc::SmartDashboard::PutNumber("Vision Y", Pose.Y().value());
  }
  // if(result.HasTargets()){
  //   units::second_t timeStamp{result.GetTimestamp()};
  //   target = result.GetBestTarget();
  //   targetID = target.GetFiducialId();   
  //   auto targetPose = aprilTags.get()->GetTagPose(targetID);
  //   frc::Transform3d camToTarget = target.GetBestCameraToTarget();
  //   frc::Pose3d camPose = targetPose.value().TransformBy(camToTarget.Inverse()); 
  //   frc::Pose3d visionMeasurement = camPose.TransformBy(robotToCam.Inverse());
  //   frc::SmartDashboard::PutNumber("VisionMesX", visionMeasurement.X().value());
  //   frc::SmartDashboard::PutNumber("VisionMesY", visionMeasurement.Y().value());  
    // frc::SmartDashboard::PutNumber( "PoseAmbiguity", target.GetPoseAmbiguity());
    // frc::SmartDashboard::PutNumber("ToTarget", camToTarget.X().value());
    // frc::SmartDashboard::PutNumber("ID DETECTED", targetID);

  // }


  
}
