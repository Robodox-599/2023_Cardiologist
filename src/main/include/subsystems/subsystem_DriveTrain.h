// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include "SwerveModule.h"
#include "Constants.h"
#include <vector>
#include <units/velocity.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/SwerveDriveOdometry.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <frc/estimator/SwerveDrivePoseEstimator.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/geometry/Translation2d.h>
#include <frc/controller/PIDController.h>
#include <frc/controller/ProfiledPIDController.h>
#include <ctre/phoenix/sensors/WPI_Pigeon2.h>
#include <frc/Timer.h>
#include <ctre/phoenix/led/CANdle.h>
#include <frc/DriverStation.h>
#include <frc2/command/CommandPtr.h>

#include <frc2/command/SubsystemBase.h>
// #include <photonlib/PhotonCamera.h>
// #include <photonlib/PhotonUtils.h>
// #include <photonlib/PhotonTrackedTarget.h>
// #include <photonlib/RobotPoseEstimator.h>
// #include <photonlib/PhotonPoseEstimator.h>

#include <frc/apriltag/AprilTagFieldLayout.h>
#include <frc/apriltag/AprilTagPoseEstimator.h>
#include <frc/apriltag/AprilTag.h>
#include <frc/apriltag/AprilTagFields.h>
#include <frc/DriverStation.h>
#include <frc/Timer.h>
#include <frc/smartdashboard/SendableChooser.h>




class subsystem_DriveTrain : public frc2::SubsystemBase {
 public:
  subsystem_DriveTrain();
  void SwerveDrive(units::meters_per_second_t xSpeed,
                   units::meters_per_second_t ySpeed,
                   units::radians_per_second_t zRot,
                   bool FieldRelative, 
                   bool IsOpenLoop);

  void SetModuleStates(wpi::array<frc::SwerveModuleState, 4> desiredStates);
  double SetThrottle(double input);
  void ChangeThrottle();
  frc2::CommandPtr ToggleThrottleCommand();
  void ResetModulesToAbsolute();
  units::meters_per_second_t AddPitchCorrection();
  units::meters_per_second_t AddRollCorrection();
  


  void ZeroGyro();

  frc2::CommandPtr ZeroGyroCommand();
  void ResetOdometry(frc::Pose2d Pose);
  void ImplementVisionPose(std::pair<frc::Pose2d, units::millisecond_t> pair);
  frc::Pose2d GetPose();
  frc::Rotation2d GetYaw();
  frc::Rotation2d GetPoseYaw();
  void SetPark();
  void TogglePark();
  bool IsPark();

  void ToggleBalanceCorrection();
  units::meters_per_second_t CalculatePitch();
  units::meters_per_second_t CalculateRoll();
  void EnableBalanceCorrection();
  void DisableBalanceCorrection();
  bool IsBalanced();

  std::pair<units::meter_t, units::meter_t> ReflectAlliance();
  
  void SetAutoOrient(bool IsOrientFront, bool IsOrientBack, double RotVelocity);
  units::radians_per_second_t GetAngularVelocity();

  std::pair<frc::Pose2d, units::second_t> getEstimatedGlobalPose();
  std::pair<frc::Pose3d, units::time::second_t> getInitialPose();
  int getID();
  bool HasAcceptableTargets();
  frc::DriverStation::Alliance GetAlliance();

  // auto GetChassisSpeed(auto chassisSpeed);
  // void SetAngleToHoloRotation(frc::Rotation2d holo);

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
  
  ctre::phoenix::sensors::WPI_Pigeon2 m_Gyro;

  SwerveModule m_FrontLeftModule;
  SwerveModule m_FrontRightModule;
  SwerveModule m_BackLeftModule;
  SwerveModule m_BackRightModule;

  SwerveConstants::Throttle DegreeOfThrottle;

  frc::SwerveDriveKinematics<4> m_kinematics{
      SwerveConstants::m_FrontLeft, SwerveConstants::m_FrontRight, SwerveConstants::m_BackLeft,
      SwerveConstants::m_BackRight};


  frc::SwerveDrivePoseEstimator<4> m_PoseEstimator;

  frc2::PIDController m_PitchCorrectionPID;
  frc2::PIDController m_RollCorrectionPID;
  bool m_IsBalancing;

  bool m_IsAutoOrient;
  DPAD::ORIENTATION m_Dpad = DPAD::ORIENTATION::NON_ORIENTED;
  int m_OrientCounter = 0;
  
  frc::PIDController m_AutoOrientPID{0.05, 
                                        0.0, 
                                        0.001};

  frc::TrapezoidProfile<units::degree>::Constraints m_constraints{AutoConstants::MaxAngularSpeed,
                                                                  AutoConstants::MaxAngularAccel};
  // frc::ProfiledPIDController<units::degree> m_ProfiledOrientPID{AutoConstants::AngleKP, 0.0, AutoConstants::AngleKD,
  //                                                        m_constraints};

  frc::ProfiledPIDController<units::radians> m_ProfiledOrientPID{175.0, 0.0, 0.0,  frc::ProfiledPIDController<units::radians>::Constraints{AutoConstants::MaxAngularSpeed, 
                                                                                                                              AutoConstants::MaxAngularAccel}};

  bool m_IsPark = false;

//   photonlib::PhotonCamera m_Camera{"Global_Shutter_Camera"};
//   photonlib::PhotonPipelineResult m_CamResult;
//   frc::Transform3d robotToCam =
//       frc::Transform3d(frc::Translation3d(16.75_in, -7_in, 14.5_in),
//                        frc::Rotation3d(0_rad, 0_rad, 0_rad));
//   frc::AprilTagFieldLayout TagLayout{frc::LoadAprilTagLayoutField(frc::AprilTagField::k2023ChargedUp)};
//   photonlib::PhotonPoseEstimator m_VisionPoseEstimator{ TagLayout, photonlib::PoseStrategy::CLOSEST_TO_REFERENCE_POSE, std::move(m_Camera), robotToCam };



/*_______________________________________________________________________________
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
  */

};
