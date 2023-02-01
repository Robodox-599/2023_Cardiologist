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
#include <frc/Timer.h>

class subsystem_PoseTracker : public frc2::SubsystemBase
{
public:
  subsystem_PoseTracker();
  std::pair<frc::Pose2d, units::millisecond_t> getEstimatedGlobalPose();
  std::pair<frc::Pose3d, units::time::second_t> getInitialPose();
  int getID();
  bool hasTarget();
  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
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
      {8, frc::Pose3d(units::meter_t(0), units::meter_t(0), units::meter_t(0),
                      frc::Rotation3d())},
      {3, frc::Pose3d(units::meter_t(0), units::meter_t(0), units::meter_t(0),
                      frc::Rotation3d( units::radian_t(0), units::radian_t(0), units::degree_t(180) ))}};
                      
  std::shared_ptr<frc::AprilTagFieldLayout> aprilTags =
      std::make_shared<frc::AprilTagFieldLayout>(tags, 54_ft, 27_ft);

  // Forward Camera
  std::shared_ptr<photonlib::PhotonCamera> cameraOne =
      std::make_shared<photonlib::PhotonCamera>("Global_Shutter_Camera");
  // Camera is mounted facing forward, half a meter forward of center, half a
  // meter up from center.
  frc::Transform3d robotToCam =
      frc::Transform3d(frc::Translation3d(11.675_in, 0_in, 30.3175_in),
                       frc::Rotation3d(0_rad, 0_rad, 0_rad));

  // ... Add other cameras here

  // Assemble the list of cameras & mount locations
  std::vector<std::pair<std::shared_ptr<photonlib::PhotonCamera>, frc::Transform3d>> cameras;
  photonlib::RobotPoseEstimator estimator{aprilTags, photonlib::CLOSEST_TO_REFERENCE_POSE, cameras};



};