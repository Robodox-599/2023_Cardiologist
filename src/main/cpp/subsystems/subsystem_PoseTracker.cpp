// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/subsystem_PoseTracker.h"
#include "frc/smartdashboard/SmartDashboard.h"

subsystem_PoseTracker::subsystem_PoseTracker()
{


  if(frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kBlue){

    m_Field.SetOrigin(frc::AprilTagFieldLayout::OriginPosition::kBlueAllianceWallRightSide);

  }else if(frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kRed){

    m_Field.SetOrigin(frc::AprilTagFieldLayout::OriginPosition::kRedAllianceWallRightSide);
    
  }else{
    printf("INVALID ALLIANCE INVALID ALLIANCE");
  }


}

   std::pair<frc::Pose2d, units::second_t>  subsystem_PoseTracker::GetEstimatedGlobalPose(){
    m_Screenshot = m_Camera.GetLatestResult();
      if(m_Screenshot.HasTargets()){
        m_Target = m_Screenshot.GetBestTarget();
        if(m_Target.GetPoseAmbiguity() < 0.2){
          int TargetID = m_Target.GetFiducialId();   
          auto TargetPose = m_Field.GetTagPose(TargetID);
          frc::Transform3d CamToTarget = m_Target.GetBestCameraToTarget();
          frc::Pose3d camPose = TargetPose.value().TransformBy(CamToTarget.Inverse()); 
          frc::Pose3d visionMeasurement = camPose.TransformBy(m_RobotToCam.Inverse());
          return std::make_pair(frc::Pose2d{visionMeasurement.ToPose2d()}, units::second_t{m_Screenshot.GetTimestamp()});
        }
      }
      return std::make_pair(frc::Pose2d{}, units::second_t{-1.0});
  }







// This method will be called once per scheduler run
void subsystem_PoseTracker::Periodic()
{



  
}
