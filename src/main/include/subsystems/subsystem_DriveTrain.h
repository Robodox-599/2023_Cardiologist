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
  void ResetModulesToAbsolute();
  units::meters_per_second_t AddPitchCorrection();
  units::meters_per_second_t AddRollCorrection();
  


  void ZeroGyro();
  void ResetOdometry(frc::Rotation2d Rotation, frc::Pose2d Pose);
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

  std::pair<units::meter_t, units::meter_t> ReflectAlliance();
  

  void SetAutoOrient(bool IsOrientFront, bool IsOrientBack, double RotVelocity);
  units::radians_per_second_t GetAngularVelocity();
  

  void SetPurpleLED();
  void SetYellowLED();
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
  int m_Dpad = Orientation::NON_ORIENTED;
  int m_OrientCounter = 0;
  
  frc::PIDController m_AutoOrientPID{0.05, 
                                        0.0, 
                                        0.001};

  ctre::phoenix::led::CANdle m_CANdle;
  // frc::Timer m_LEDTimer;
  SwerveConstants::LEDState m_LEDState = SwerveConstants::LEDState::Standby;
  bool m_IsPark = false;

  

  

  
                           
  
};
