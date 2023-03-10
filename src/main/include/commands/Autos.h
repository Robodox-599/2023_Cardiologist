// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.



#pragma once

#include <frc2/command/CommandPtr.h>
#include "Constants.h"
#include "command_DriveAuton.h"
#include "command_ArmInputSwitch.h"
#include "command_IntakeObject.h"
#include "cGroup_Arm.h"
#include "command_MoveArmManually.h"
#include "command_OuttakeObject.h"
#include "command_ToggleClamp.h"
#include "subsystems/subsystem_PoseTracker.h"
#include "subsystems/subsystem_DriveTrain.h"
#include "subsystems/subsystem_Arm.h"
#include "subsystems/subsystem_Intake.h"
#include "subsystems/ExampleSubsystem.h"


namespace autos {
    
    frc2::CommandPtr TestAuto(subsystem_DriveTrain* DriveTrain, subsystem_PoseTracker* PoseTracker);

    frc2::CommandPtr autos::OneSTIS_0(subsystem_Intake* Intake, subsystem_Arm* Arm);
    frc2::CommandPtr autos::OneSTIS_1(subsystem_DriveTrain* DriveTrain, subsystem_PoseTracker* PoseTracker, subsystem_Intake* Intake, subsystem_Arm* Arm);
    frc2::CommandPtr autos::OneSTIS_2(subsystem_DriveTrain* DriveTrain, subsystem_PoseTracker* PoseTracker);
    frc2::CommandPtr autos::OneSTIS_3(subsystem_Intake* Intake, subsystem_Arm* Arm);

    frc2::CommandPtr autos::TwoSISC_0(subsystem_Intake* Intake, subsystem_Arm* Arm);
    frc2::CommandPtr autos::TwoSISC_1(subsystem_DriveTrain* DriveTrain, subsystem_PoseTracker* PoseTracker, subsystem_Intake* Intake, subsystem_Arm* Arm);
    frc2::CommandPtr autos::TwoSISC_2(subsystem_DriveTrain* DriveTrain, subsystem_PoseTracker* PoseTracker);
    frc2::CommandPtr autos::TwoSISC_2_half(subsystem_Intake* Intake, subsystem_Arm* Arm);
    frc2::CommandPtr autos::TwoSISC_3(subsystem_DriveTrain* DriveTrain, subsystem_PoseTracker* PoseTracker, subsystem_Arm* Arm);

    frc2::CommandPtr autos::ThreeSISIS_0(subsystem_Intake* Intake, subsystem_Arm* Arm);
    frc2::CommandPtr autos::ThreeSISIS_1(subsystem_DriveTrain* DriveTrain, subsystem_PoseTracker* PoseTracker, subsystem_Intake* Intake, subsystem_Arm* Arm);
    frc2::CommandPtr autos::ThreeSISIS_2(subsystem_DriveTrain* DriveTrain, subsystem_PoseTracker* PoseTracker);
    frc2::CommandPtr autos::ThreeSISIS_2_half(subsystem_Intake* Intake, subsystem_Arm* Arm);
    frc2::CommandPtr autos::ThreeSISIS_3(subsystem_DriveTrain* DriveTrain, subsystem_PoseTracker* PoseTracker, subsystem_Intake* Intake, subsystem_Arm* Arm);
    frc2::CommandPtr autos::ThreeSISIS_4(subsystem_DriveTrain* DriveTrain, subsystem_PoseTracker* PoseTracker);
    frc2::CommandPtr autos::ThreeSISIS_5(subsystem_Intake* Intake, subsystem_Arm* Arm);

    frc2::CommandPtr autos::ThreeSTIS_0(subsystem_Intake* Intake, subsystem_Arm* Arm);
    frc2::CommandPtr autos::ThreeSTIS_1(subsystem_DriveTrain* DriveTrain, subsystem_PoseTracker* PoseTracker, subsystem_Intake* Intake, subsystem_Arm* Arm);
    frc2::CommandPtr autos::ThreeSTIS_2(subsystem_DriveTrain* DriveTrain, subsystem_PoseTracker* PoseTracker);
    frc2::CommandPtr autos::ThreeSTIS_3(subsystem_Intake* Intake, subsystem_Arm* Arm);

    frc2::CommandPtr autos::TwoSTISC_0(subsystem_Intake* Intake, subsystem_Arm* Arm);
    frc2::CommandPtr autos::TwoSTISC_1(subsystem_DriveTrain* DriveTrain, subsystem_PoseTracker* PoseTracker, subsystem_Intake* Intake, subsystem_Arm* Arm);
    frc2::CommandPtr autos::TwoSTISC_2(subsystem_DriveTrain* DriveTrain, subsystem_PoseTracker* PoseTracker);
    frc2::CommandPtr autos::TwoSTISC_3(subsystem_DriveTrain* DriveTrain, subsystem_PoseTracker* PoseTracker, subsystem_Intake* Intake, subsystem_Arm* Arm);

    frc2::CommandPtr autos::OneSISIS_0(subsystem_Intake* Intake, subsystem_Arm* Arm);
    frc2::CommandPtr autos::OneSISIS_1(subsystem_DriveTrain* DriveTrain, subsystem_PoseTracker* PoseTracker, subsystem_Intake* Intake, subsystem_Arm* Arm);
    frc2::CommandPtr autos::OneSISIS_2(subsystem_DriveTrain* DriveTrain, subsystem_PoseTracker* PoseTracker);
    frc2::CommandPtr autos::OneSISIS_2_half(subsystem_Intake* Intake, subsystem_Arm* Arm);
    frc2::CommandPtr autos::OneSISIS_3(subsystem_DriveTrain* DriveTrain, subsystem_PoseTracker* PoseTracker, subsystem_Intake* Intake, subsystem_Arm* Arm);
    frc2::CommandPtr autos::OneSISIS_4(subsystem_DriveTrain* DriveTrain, subsystem_PoseTracker* PoseTracker);
    frc2::CommandPtr autos::OneSISIS_5(subsystem_Intake* Intake, subsystem_Arm* Arm);
}

namespace paths{
  frc2::CommandPtr paths::OneSTIS(subsystem_DriveTrain* DriveTrain, subsystem_PoseTracker* PoseTracker, subsystem_Intake* Intake, subsystem_Arm* Arm);
  frc2::CommandPtr paths::TwoSISC(subsystem_DriveTrain* DriveTrain, subsystem_PoseTracker* PoseTracker, subsystem_Intake* Intake, subsystem_Arm* Arm);
  frc2::CommandPtr paths::ThreeSISIS(subsystem_DriveTrain* DriveTrain, subsystem_PoseTracker* PoseTracker, subsystem_Intake* Intake, subsystem_Arm* Arm);
  frc2::CommandPtr paths::ThreeSTIS(subsystem_DriveTrain* DriveTrain, subsystem_PoseTracker* PoseTracker, subsystem_Intake* Intake, subsystem_Arm* Arm);
  frc2::CommandPtr paths::TwoSTISC(subsystem_DriveTrain* DriveTrain, subsystem_PoseTracker* PoseTracker, subsystem_Intake* Intake, subsystem_Arm* Arm);
  frc2::CommandPtr paths::OneSISIS(subsystem_DriveTrain* DriveTrain, subsystem_PoseTracker* PoseTracker, subsystem_Intake* Intake, subsystem_Arm* Arm);
}
  // namespace autos