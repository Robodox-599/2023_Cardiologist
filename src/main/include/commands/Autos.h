// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandPtr.h>
#include "command_DriveAuton.h"
#include "subsystems/subsystem_PoseTracker.h"
#include "subsystems/ExampleSubsystem.h"

namespace autos {
/**
 * Example static factory for an autonomous command.
 */


frc2::CommandPtr TestAuto(subsystem_DriveTrain* DriveTrain, subsystem_PoseTracker* PoseTracker);


//Two Ball Auto
frc2::CommandPtr Niemann(subsystem_DriveTrain* DriveTrain, subsystem_PoseTracker* PoseTracker);


//Taxi Auto
frc2::CommandPtr Kasparov(subsystem_DriveTrain* DriveTrain, subsystem_PoseTracker* PoseTracker);

}  // namespace autos


