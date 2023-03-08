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
/**
 * Example static factory for an autonomous command.
 */


frc2::CommandPtr TestAuto(subsystem_DriveTrain* DriveTrain, subsystem_PoseTracker* PoseTracker);

/*
//Two Ball Auto

//Niemann
frc2::CommandPtr Yes2(subsystem_DriveTrain* DriveTrain, subsystem_PoseTracker* PoseTracker);

//Hikaru
frc2::CommandPtr FourTC(subsystem_DriveTrain* DriveTrain, subsystem_Arm* Arm, subsystem_Intake* Intake, subsystem_PoseTracker* PoseTracker);

//Fischer
frc2::CommandPtr ThreeST(subsystem_DriveTrain* DriveTrain, subsystem_Arm* Arm, subsystem_Intake* Intake);

//Magnus
frc2::CommandPtr TwoSTC(subsystem_DriveTrain* DriveTrain, subsystem_Arm* Arm, subsystem_Intake* Intake, subsystem_PoseTracker* PoseTracker);

//Giri
frc2::CommandPtr ThreeDST(subsystem_DriveTrain* DriveTrain, subsystem_Arm* Arm, subsystem_Intake* Intake, subsystem_PoseTracker* PoseTracker);

//Kasparov
frc2::CommandPtr FourT(subsystem_DriveTrain* DriveTrain, subsystem_PoseTracker* PoseTracker);

frc2::CommandPtr Giri(subsystem_DriveTrain* DriveTrain, subsystem_Arm* Arm, subsystem_Intake* Intake, subsystem_PoseTracker* PoseTracker);


//Taxi Auto
frc2::CommandPtr Kasparov(subsystem_DriveTrain* DriveTrain, subsystem_PoseTracker* PoseTracker);
*/

//Good CMDs
frc2::CommandPtr OneSTIS_0(subsystem_Intake* Intake, subsystem_Arm* Arm);
frc2::CommandPtr OneSTIS_1(subsystem_DriveTrain* DriveTrain, subsystem_PoseTracker* PoseTracker, subsystem_Intake* Intake, subsystem_Arm* Arm);
frc2::CommandPtr OneSTIS_2(subsystem_DriveTrain* DriveTrain, subsystem_PoseTracker* PoseTracker, subsystem_Arm* Arm);
frc2::CommandPtr OneSTIS_3(subsystem_Intake* Intake);

frc2::CommandPtr TwoSISC_0(subsystem_Intake* Intake, subsystem_Arm* Arm);
frc2::CommandPtr TwoSISC_1(subsystem_DriveTrain* DriveTrain, subsystem_PoseTracker* PoseTracker, subsystem_Intake* Intake, subsystem_Arm* Arm);
frc2::CommandPtr TwoSISC_2(subsystem_DriveTrain* DriveTrain, subsystem_PoseTracker* PoseTracker, subsystem_Arm* Arm);
frc2::CommandPtr TwoSISC_2_half(subsystem_Intake* Intake);
frc2::CommandPtr TwoSISC_3(subsystem_DriveTrain* DriveTrain, subsystem_PoseTracker* PoseTracker, subsystem_Arm* Arm);

frc2::CommandPtr ThreeSISIS_0(subsystem_Intake* Intake, subsystem_Arm* Arm);
frc2::CommandPtr ThreeSISIS_1(subsystem_DriveTrain* DriveTrain, subsystem_PoseTracker* PoseTracker, subsystem_Intake* Intake, subsystem_Arm* Arm);
frc2::CommandPtr ThreeSISIS_2(subsystem_DriveTrain* DriveTrain, subsystem_PoseTracker* PoseTracker, subsystem_Arm* Arm);
frc2::CommandPtr ThreeSISIS_2_half(subsystem_Intake* Intake);
frc2::CommandPtr ThreeSISIS_3(subsystem_DriveTrain* DriveTrain, subsystem_PoseTracker* PoseTracker, subsystem_Intake* Intake, subsystem_Arm* Arm);
frc2::CommandPtr ThreeSISIS_4(subsystem_DriveTrain* DriveTrain, subsystem_PoseTracker* PoseTracker, subsystem_Arm* Arm);
frc2::CommandPtr ThreeSISIS_5(subsystem_Intake* Intake);

frc2::CommandPtr ThreeSTIS_0(subsystem_Intake* Intake, subsystem_Arm* Arm);
frc2::CommandPtr ThreeSTIS_1(subsystem_DriveTrain* DriveTrain, subsystem_PoseTracker* PoseTracker, subsystem_Intake* Intake, subsystem_Arm* Arm);
frc2::CommandPtr ThreeSTIS_2(subsystem_DriveTrain* DriveTrain, subsystem_PoseTracker* PoseTracker, subsystem_Arm* Arm);
frc2::CommandPtr ThreeSTIS_3(subsystem_Intake* Intake);

frc2::CommandPtr TwoSTISC_0(subsystem_Intake* Intake, subsystem_Arm* Arm);
frc2::CommandPtr TwoSTISC_1(subsystem_DriveTrain* DriveTrain, subsystem_PoseTracker* PoseTracker, subsystem_Intake* Intake, subsystem_Arm* Arm);
frc2::CommandPtr TwoSTISC_2(subsystem_DriveTrain* DriveTrain, subsystem_PoseTracker* PoseTracker, subsystem_Arm* Arm);
frc2::CommandPtr TwoSTISC_3(subsystem_DriveTrain* DriveTrain, subsystem_PoseTracker* PoseTracker, subsystem_Intake* Intake);

frc2::CommandPtr OneSISIS_0(subsystem_Intake* Intake, subsystem_Arm* Arm);
frc2::CommandPtr OneSISIS_1(subsystem_DriveTrain* DriveTrain, subsystem_PoseTracker* PoseTracker, subsystem_Intake* Intake, subsystem_Arm* Arm);
frc2::CommandPtr OneSISIS_2(subsystem_DriveTrain* DriveTrain, subsystem_PoseTracker* PoseTracker, subsystem_Arm* Arm);
frc2::CommandPtr OneSISIS_2_half(subsystem_Intake* Intake);
frc2::CommandPtr OneSISIS_3(subsystem_DriveTrain* DriveTrain, subsystem_PoseTracker* PoseTracker, subsystem_Intake* Intake, subsystem_Arm* Arm);
frc2::CommandPtr OneSISIS_4(subsystem_DriveTrain* DriveTrain, subsystem_PoseTracker* PoseTracker, subsystem_Arm* Arm);
frc2::CommandPtr OneSISIS_5(subsystem_Intake* Intake);

}  // namespace autos


