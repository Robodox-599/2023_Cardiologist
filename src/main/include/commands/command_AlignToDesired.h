// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/subsystem_DriveTrain.h"
#include "subsystems/subsystem_PoseTracker.h"

#include <frc/controller/PIDController.h>
#include <units/length.h>
#include <units/angle.h>


/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class command_AlignToDesired
    : public frc2::CommandHelper<frc2::CommandBase, command_AlignToDesired> {
 public:
  command_AlignToDesired(subsystem_DriveTrain* DriveTrain, subsystem_PoseTracker* PoseTracker, std::function<frc::Pose2d()> DesiredPose);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

  private:
  subsystem_DriveTrain* m_DriveTrain;
  subsystem_PoseTracker* m_PoseTracker;
  std::function<frc::Pose2d()> m_DesiredPose;
  



  frc::TrapezoidProfile<units::meter>::Constraints m_Xconstraints{AutoConstants::MaxSpeed,
                                                                  AutoConstants::MaxAccel};
  frc::ProfiledPIDController<units::meter> m_XProfiledPID{AutoConstants::XDriveKP, 0.0, AutoConstants::XDriveKD,
                                                         m_Xconstraints};

  frc::TrapezoidProfile<units::meter>::Constraints m_Yconstraints{AutoConstants::MaxSpeed,
                                                                  AutoConstants::MaxAccel};
  frc::ProfiledPIDController<units::meter> m_YProfiledPID{AutoConstants::YDriveKP, 0.0, AutoConstants::YDriveKD,
                                                         m_Yconstraints};

  frc::TrapezoidProfile<units::degree>::Constraints m_Thetaconstraints{AutoConstants::MaxAngularSpeed,
                                                                  AutoConstants::MaxAngularAccel};
  frc::ProfiledPIDController<units::degree> m_ThetaProfiledPID{AutoConstants::AngleKP, 0.0, AutoConstants::AngleKD,
                                                         m_Thetaconstraints};
};
