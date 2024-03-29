// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.



#pragma once

#include <frc2/command/CommandPtr.h>
#include "Constants.h"
#include "command_DriveAuton.h"
#include "command_AutoClamp.h"
#include "cGroup_Arm.h"
#include "command_MoveArmManually.h"
#include "subsystems/subsystem_DriveTrain.h"
#include "subsystems/subsystem_Arm.h"
#include "subsystems/subsystem_Intake.h"
#include "subsystems/subsystem_PoseTracker.h"
#include "subsystems/subsystem_LED.h"
#include "commands/command_TimeOut.h"
#include "commands/command_Balance.h"
#include "commands/Autos.h"
#include <frc2/command/WaitCommand.h>


namespace autos {

    // frc2::CommandPtr One_ThreeScoreAuto(subsystem_DriveTrain* DriveTrain, subsystem_PoseTracker* PoseTracker, subsystem_Arm* Arm, subsystem_Intake* Intake, subsystem_GroundTake* GroundTake, subsystem_LED* LED);
    // frc2::CommandPtr One_TwoScore(subsystem_DriveTrain* DriveTrain, subsystem_PoseTracker* PoseTracker, subsystem_Arm* Arm, subsystem_Intake* Intake, subsystem_GroundTake* GroundTake, subsystem_LED* LED);
    // frc2::CommandPtr One_TwoScoreAndBalance(subsystem_DriveTrain* DriveTrain, subsystem_PoseTracker* PoseTracker, subsystem_Arm* Arm, subsystem_Intake* Intake, subsystem_GroundTake* GroundTake, subsystem_LED* LED);
    // frc2::CommandPtr One_ThreeScoreAndBalance(subsystem_DriveTrain* DriveTrain, subsystem_PoseTracker* PoseTracker, subsystem_Arm* Arm, subsystem_Intake* Intake, subsystem_GroundTake* GroundTake, subsystem_LED* LED);

    // frc2::CommandPtr TestAuto(subsystem_DriveTrain* DriveTrain, subsystem_PoseTracker* PoseTracker );

   frc2::CommandPtr Taxi(subsystem_DriveTrain* DriveTrain, subsystem_PoseTracker* PoseTracker );

    // frc2::CommandPtr ScoreHighCube(subsystem_Intake* Intake, subsystem_Arm* Arm);
    // frc2::CommandPtr ScoreMidCube(subsystem_Intake* Intake, subsystem_Arm* Arm);

    // frc2::CommandPtr ScoreHighCone(subsystem_Intake* Intake, subsystem_Arm* Arm);
    // frc2::CommandPtr ScoreMidCone(subsystem_Intake* Intake, subsystem_Arm* Arm);

    frc2::CommandPtr ScoreConeAndTaxi(subsystem_DriveTrain* DriveTrain, subsystem_PoseTracker* PoseTracker, subsystem_Intake* Intake, subsystem_Arm* Arm);

    frc2::CommandPtr One_ScoreCubeAndTaxi(subsystem_DriveTrain* DriveTrain, subsystem_PoseTracker* PoseTracker, subsystem_Intake* Intake, subsystem_Arm* Arm);
    frc2::CommandPtr Two_ScoreCubeAndTaxi(subsystem_DriveTrain* DriveTrain, subsystem_PoseTracker* PoseTracker, subsystem_Intake* Intake, subsystem_Arm* Arm);
    frc2::CommandPtr Three_ScoreCubeAndTaxi(subsystem_DriveTrain* DriveTrain, subsystem_PoseTracker* PoseTracker, subsystem_Intake* Intake, subsystem_Arm* Arm);


    frc2::CommandPtr Three_TaxiAndBalance(subsystem_DriveTrain* DriveTrain, subsystem_PoseTracker* PoseTracker );
    frc2::CommandPtr Two_TaxiAndBalance(subsystem_DriveTrain* DriveTrain, subsystem_PoseTracker* PoseTracker );
    frc2::CommandPtr Two_ScoreTaxiAndBalance(subsystem_DriveTrain* DriveTrain, subsystem_PoseTracker* PoseTracker, subsystem_Intake* Intake, subsystem_Arm* Arm);

    frc2::CommandPtr One_Cone_Taxi(subsystem_DriveTrain* DriveTrain, subsystem_PoseTracker* PoseTracker, subsystem_Intake* Intake, subsystem_Arm* Arm);
    frc2::CommandPtr Two_Cone_Taxi_Balance(subsystem_DriveTrain* DriveTrain, subsystem_PoseTracker* PoseTracker, subsystem_Intake* Intake, subsystem_Arm* Arm);
    frc2::CommandPtr Three_Cone_Taxi(subsystem_DriveTrain* DriveTrain, subsystem_PoseTracker* PoseTracker, subsystem_Intake* Intake, subsystem_Arm* Arm);

}

namespace paths{
}
  // namespace autos
