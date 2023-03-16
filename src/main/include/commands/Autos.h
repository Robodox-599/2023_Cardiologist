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
#include "commands/command_TimeOut.h"
#include "commands/command_Balance.h"


namespace autos {
    
    frc2::CommandPtr TestAuto(subsystem_DriveTrain* DriveTrain, subsystem_PoseTracker* PoseTracker);

    frc2::CommandPtr Kasparov(subsystem_DriveTrain* DriveTrain, subsystem_PoseTracker* PoseTracker);

    frc2::CommandPtr OneSTIS_0(subsystem_Intake* Intake, subsystem_Arm* Arm);
    frc2::CommandPtr OneSTIS_1(subsystem_DriveTrain* DriveTrain, subsystem_PoseTracker* PoseTracker, subsystem_Intake* Intake, subsystem_Arm* Arm);
    frc2::CommandPtr OneSTIS_2(subsystem_DriveTrain* DriveTrain, subsystem_PoseTracker* PoseTracker);
    frc2::CommandPtr OneSTIS_3(subsystem_Intake* Intake, subsystem_Arm* Arm);

    frc2::CommandPtr TwoSISC_0(subsystem_Intake* Intake, subsystem_Arm* Arm);
    frc2::CommandPtr TwoSISC_1(subsystem_DriveTrain* DriveTrain, subsystem_PoseTracker* PoseTracker, subsystem_Intake* Intake, subsystem_Arm* Arm);
    frc2::CommandPtr TwoSISC_2(subsystem_DriveTrain* DriveTrain, subsystem_PoseTracker* PoseTracker);
    frc2::CommandPtr TwoSISC_2_half(subsystem_Intake* Intake, subsystem_Arm* Arm);
    frc2::CommandPtr TwoSISC_3(subsystem_DriveTrain* DriveTrain, subsystem_PoseTracker* PoseTracker, subsystem_Arm* Arm);

    frc2::CommandPtr ThreeSISIS_0(subsystem_Intake* Intake, subsystem_Arm* Arm);
    frc2::CommandPtr ThreeSISIS_1(subsystem_DriveTrain* DriveTrain, subsystem_PoseTracker* PoseTracker, subsystem_Intake* Intake, subsystem_Arm* Arm);
    frc2::CommandPtr ThreeSISIS_2(subsystem_DriveTrain* DriveTrain, subsystem_PoseTracker* PoseTracker);
    frc2::CommandPtr ThreeSISIS_2_half(subsystem_Intake* Intake, subsystem_Arm* Arm);
    frc2::CommandPtr ThreeSISIS_3(subsystem_DriveTrain* DriveTrain, subsystem_PoseTracker* PoseTracker, subsystem_Intake* Intake, subsystem_Arm* Arm);
    frc2::CommandPtr ThreeSISIS_4(subsystem_DriveTrain* DriveTrain, subsystem_PoseTracker* PoseTracker);
    frc2::CommandPtr ThreeSISIS_5(subsystem_Intake* Intake, subsystem_Arm* Arm);

    frc2::CommandPtr ThreeSTIS_0(subsystem_Intake* Intake, subsystem_Arm* Arm);
    frc2::CommandPtr ThreeSTIS_1(subsystem_DriveTrain* DriveTrain, subsystem_PoseTracker* PoseTracker, subsystem_Intake* Intake, subsystem_Arm* Arm);
    frc2::CommandPtr ThreeSTIS_2(subsystem_DriveTrain* DriveTrain, subsystem_PoseTracker* PoseTracker);
    frc2::CommandPtr ThreeSTIS_3(subsystem_Intake* Intake, subsystem_Arm* Arm);

    frc2::CommandPtr TwoSTISC_0(subsystem_Intake* Intake, subsystem_Arm* Arm);
    frc2::CommandPtr TwoSTISC_1(subsystem_DriveTrain* DriveTrain, subsystem_PoseTracker* PoseTracker, subsystem_Intake* Intake, subsystem_Arm* Arm);
    frc2::CommandPtr TwoSTISC_2(subsystem_DriveTrain* DriveTrain, subsystem_PoseTracker* PoseTracker);
    frc2::CommandPtr TwoSTISC_3(subsystem_DriveTrain* DriveTrain, subsystem_PoseTracker* PoseTracker, subsystem_Intake* Intake, subsystem_Arm* Arm);

    frc2::CommandPtr OneSISIS_0(subsystem_Intake* Intake, subsystem_Arm* Arm);
    frc2::CommandPtr OneSISIS_1(subsystem_DriveTrain* DriveTrain, subsystem_PoseTracker* PoseTracker, subsystem_Intake* Intake, subsystem_Arm* Arm);
    frc2::CommandPtr OneSISIS_2(subsystem_DriveTrain* DriveTrain, subsystem_PoseTracker* PoseTracker);
    frc2::CommandPtr OneSISIS_2_half(subsystem_Intake* Intake, subsystem_Arm* Arm);
    frc2::CommandPtr OneSISIS_3(subsystem_DriveTrain* DriveTrain, subsystem_PoseTracker* PoseTracker, subsystem_Intake* Intake, subsystem_Arm* Arm);
    frc2::CommandPtr OneSISIS_4(subsystem_DriveTrain* DriveTrain, subsystem_PoseTracker* PoseTracker);
    frc2::CommandPtr OneSISIS_5(subsystem_Intake* Intake, subsystem_Arm* Arm);

    frc2::CommandPtr OneSTIS(subsystem_DriveTrain* DriveTrain, subsystem_PoseTracker* PoseTracker, subsystem_Intake* Intake, subsystem_Arm* Arm);
    frc2::CommandPtr TwoSISC(subsystem_DriveTrain* DriveTrain, subsystem_PoseTracker* PoseTracker, subsystem_Intake* Intake, subsystem_Arm* Arm);
    frc2::CommandPtr ThreeSISIS(subsystem_DriveTrain* DriveTrain, subsystem_PoseTracker* PoseTracker, subsystem_Intake* Intake, subsystem_Arm* Arm);
    frc2::CommandPtr ThreeSTIS(subsystem_DriveTrain* DriveTrain, subsystem_PoseTracker* PoseTracker, subsystem_Intake* Intake, subsystem_Arm* Arm);
    frc2::CommandPtr TwoSTISC(subsystem_DriveTrain* DriveTrain, subsystem_PoseTracker* PoseTracker, subsystem_Intake* Intake, subsystem_Arm* Arm);
    frc2::CommandPtr OneSISIS(subsystem_DriveTrain* DriveTrain, subsystem_PoseTracker* PoseTracker, subsystem_Intake* Intake, subsystem_Arm* Arm);

    frc2::CommandPtr ScoreAndTaxi(subsystem_DriveTrain* DriveTrain, subsystem_PoseTracker* PoseTracker, subsystem_Intake* Intake, subsystem_Arm* Arm);
    frc2::CommandPtr TaxiAndBalance(subsystem_DriveTrain* DriveTrain, subsystem_PoseTracker* PoseTracker);
}

namespace paths{
}
  // namespace autos
