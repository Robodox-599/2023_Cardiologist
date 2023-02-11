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

frc2::CommandPtr autos::TestAuto(subsystem_DriveTrain* DriveTrain, subsystem_PoseTracker* PoseTracker){
  return command_DriveAuton(DriveTrain, PoseTracker, "LinePath",  true).ToPtr();
}


frc2::CommandPtr autos::Niemann(subsystem_DriveTrain* DriveTrain, subsystem_PoseTracker* PoseTracker){
  return frc2::cmd::Sequence( command_DriveAuton(DriveTrain, PoseTracker, "Niemann (Yes^2)", true).ToPtr(), 
                              command_DriveAuton(DriveTrain, PoseTracker, "Niemann 2 (Yes^2)", false).ToPtr(),
                              command_DriveAuton(DriveTrain, PoseTracker, "Niemann 3 (Yes^2)", false).ToPtr());
}

frc2::CommandPtr autos::Kasparov(subsystem_DriveTrain* DriveTrain, subsystem_PoseTracker* PoseTracker){
  return frc2::cmd::Sequence( command_DriveAuton(DriveTrain, PoseTracker, "Kasparov (4T)", true).ToPtr() );
}

