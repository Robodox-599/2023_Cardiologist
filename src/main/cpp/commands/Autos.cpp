// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


#include <frc2/command/Commands.h>

#include "commands/ExampleCommand.h"
#include "commands/Autos.h"

frc2::CommandPtr autos::ExampleAuto(ExampleSubsystem* subsystem) {
  return frc2::cmd::Sequence(subsystem->ExampleMethodCommand(),
                             ExampleCommand(subsystem).ToPtr());
}

frc2::CommandPtr autos::TestAuto(subsystem_DriveTrain* DriveTrain, subsystem_PoseTracker* PoseTracker, frc::DriverStation::Alliance AllianceColor){
  return command_DriveAuton(DriveTrain, PoseTracker, "LinePath", AllianceColor, true).ToPtr();
}
