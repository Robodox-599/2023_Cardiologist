// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once



#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include <frc/controller/HolonomicDriveController.h>
#include <frc2/command/SwerveControllerCommand.h>
#include <frc/controller/PIDController.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <frc/trajectory/Trajectory.h>
#include <frc/trajectory/TrajectoryConfig.h>
#include <frc/trajectory/TrajectoryUtil.h>
#include <frc/trajectory/TrajectoryGenerator.h>
#include <frc/interfaces/Gyro.h>
#include <frc/Filesystem.h>
#include <frc/Timer.h>
#include <wpi/fs.h>
#include <cmath>
#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/velocity.h>
#include "subsystems/subsystem_DriveTrain.h"
#include "subsystems/subsystem_PoseTracker.h"
#include <pathplanner/lib/PathPlanner.h>
#include <pathplanner/lib/controllers/PPHolonomicDriveController.h>
#include <frc/Timer.h>
/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */

class command_DriveAuton: 
  public frc2::CommandHelper<frc2::CommandBase, command_DriveAuton> {
    public:
      command_DriveAuton(subsystem_DriveTrain* DriveTrain, subsystem_PoseTracker* PoseTracker, std::string TrajFilePath, bool ToReset);

      void Initialize() override;

      void Execute() override;

      void End(bool interrupted) override;

      bool IsFinished() override;

    private:
      subsystem_DriveTrain* m_DriveTrain;
      subsystem_PoseTracker* m_PoseTracker;
      pathplanner::PathPlannerTrajectory m_Trajectory;
      bool m_ToReset;
      frc::HolonomicDriveController m_DriveController;
      frc::Timer m_Timer{};

      



};
