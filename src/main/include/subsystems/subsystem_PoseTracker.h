// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <photonlib/PhotonCamera.h>
#include <photonlib/PhotonUtils.h>
#include <photonlib/PhotonTrackedTarget.h>
#include <photonlib/RobotPoseEstimator.h>

#include <frc/apriltag/AprilTagFieldLayout.h>
#include <frc/apriltag/AprilTagPoseEstimator.h>
#include <frc/apriltag/AprilTag.h>
#include <frc/DriverStation.h>
#include <frc/Timer.h>
#include <frc/smartdashboard/SendableChooser.h>

class subsystem_PoseTracker : public frc2::SubsystemBase
{
public:
  subsystem_PoseTracker();
  std::pair<frc::Pose2d, units::second_t> getEstimatedGlobalPose();
  std::pair<frc::Pose3d, units::time::second_t> getInitialPose();
  int getID();
  bool HasAcceptableTargets();
  frc::DriverStation::Alliance GetAlliance();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
  void SetAllianceOrigin(frc::DriverStation::Alliance AllianceColor);



  photonlib::PhotonCamera camera{"Global_Shutter_Camera"};
  photonlib::PhotonPipelineResult result = camera.GetLatestResult();
  bool hasTargets = result.HasTargets();
  photonlib::PhotonTrackedTarget target = result.GetBestTarget();
  // wpi::ArrayRef<photonlib::PhotonTrackedTarget> targets = result.GetTargets();
  double yaw = target.GetYaw();
  double pitch = target.GetPitch();
  double area = target.GetArea();
  int targetID = target.GetFiducialId();
  double poseAmbiguity = target.GetPoseAmbiguity();
  frc::Transform3d bestCameraToTarget = target.GetBestCameraToTarget();
  frc::Transform3d alternateCameraToTarget = target.GetAlternateCameraToTarget();

  std::vector<frc::AprilTag> tags = {
      {1, frc::Pose3d( 610.77_in, 42.19_in, 18.22_in,
                      frc::Rotation3d( 0_deg, 0_deg, 180_deg ))},

      {2, frc::Pose3d( 610.77_in, 108.19_in, 18.22_in,
                      frc::Rotation3d( 0_deg, 0_deg, 180_deg ))},

      {3, frc::Pose3d( 610.77_in, 174.19_in, 18.22_in,
                      frc::Rotation3d( 0_deg, 0_deg, 180_deg ))}, 

      {4, frc::Pose3d( 636.96_in, 265.74_in, 27.38_in,
                    frc::Rotation3d( 0_deg, 0_deg, 180_deg ))},  

      {5, frc::Pose3d( 14.25_in, 265.74_in, 27.38_in,
                    frc::Rotation3d( 0_deg, 0_deg, 0_deg ))}, 

      {6, frc::Pose3d( 40.45_in, 174.19_in, 18.22_in,
                    frc::Rotation3d( 0_deg, 0_deg, 0_deg ))},

      {7, frc::Pose3d( 40.45_in, 108.19_in, 18.22_in,
                    frc::Rotation3d( 0_deg, 0_deg, 0_deg ))}, 

      {8, frc::Pose3d( 40.45_in, 42.19_in, 18.22_in,
                    frc::Rotation3d( 0_deg, 0_deg, 0_deg ))}   
                      };
                      
  std::shared_ptr<frc::AprilTagFieldLayout> aprilTags =
      std::make_shared<frc::AprilTagFieldLayout>(tags, 651.25_in, 315.5_in);

  // Forward Camera
  std::shared_ptr<photonlib::PhotonCamera> cameraOne =
      std::make_shared<photonlib::PhotonCamera>("Global_Shutter_Camera");
  // Camera is mounted facing forward, half a meter forward of center, half a
  // meter up from center.
  frc::Transform3d robotToCam =
      frc::Transform3d(frc::Translation3d(16.75_in, -7_in, 14.5_in),
                       frc::Rotation3d(0_rad, 0_rad, 0_rad));

  // ... Add other cameras here

  // Assemble the list of cameras & mount locations
  std::vector<std::pair<std::shared_ptr<photonlib::PhotonCamera>, frc::Transform3d>> cameras;
  photonlib::RobotPoseEstimator estimator{aprilTags, photonlib::CLOSEST_TO_REFERENCE_POSE, cameras};

  frc::DriverStation::Alliance m_Alliance{ frc::DriverStation::Alliance::kBlue };
  frc::SendableChooser<frc::DriverStation::Alliance> m_Chooser{};
  const frc::DriverStation::Alliance BLUE_ALLIANCE = frc::DriverStation::Alliance::kBlue;
  const frc::DriverStation::Alliance RED_ALLIANCE = frc::DriverStation::Alliance::kRed;

};