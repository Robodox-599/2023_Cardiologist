// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandPtr.h>

#include "subsystems/subsystem_DriveTrain.h"
#include "commands/command_DriveAuton.h"

#include <frc/trajectory/Trajectory.h>
#include <frc/trajectory/TrajectoryConfig.h>
#include <frc/trajectory/TrajectoryUtil.h>
#include <frc/trajectory/TrajectoryGenerator.h>
#include <frc/Filesystem.h>
#include <wpi/fs.h>

#include "subsystems/ExampleSubsystem.h"

namespace autos {
/**
 * Example static factory for an autonomous command.
 */
frc2::CommandPtr ExampleAuto(ExampleSubsystem* subsystem);
frc2::CommandPtr TestAuto(subsystem_DriveTrain* DriveTrain, std::string TrajFilePath, bool ToReset);
}  // namespace autos
