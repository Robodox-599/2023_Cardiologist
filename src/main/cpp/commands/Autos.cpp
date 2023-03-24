// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/Autos.h"

#include <frc2/command/Commands.h>

#include "commands/ExampleCommand.h"



frc2::CommandPtr autos::TestAuto(subsystem_DriveTrain* DriveTrain, subsystem_PoseTracker* PoseTracker){
  return command_DriveAuton(DriveTrain, PoseTracker, "Test",  true, false).ToPtr();
}

/*
frc2::CommandPtr autos::Niemann(subsystem_DriveTrain* DriveTrain, subsystem_PoseTracker* PoseTracker){
  return frc2::cmd::Sequence( command_DriveAuton(DriveTrain, PoseTracker, "Niemann (Yes^2)", true, false).ToPtr(), 
                              command_DriveAuton(DriveTrain, PoseTracker, "Niemann 2 (Yes^2)", false).ToPtr(),
                              command_DriveAuton(DriveTrain, PoseTracker, "Niemann 3 (Yes^2)", false).ToPtr());
}
*/
frc2::CommandPtr autos::Kasparov(subsystem_DriveTrain* DriveTrain, subsystem_PoseTracker* PoseTracker){
  return frc2::cmd::Sequence( command_DriveAuton(DriveTrain, PoseTracker, "Kasparov (4T)", true, false).ToPtr() );
}


frc2::CommandPtr autos::ScoreHighCube(subsystem_Intake* Intake, subsystem_Arm* Arm){
  return frc2::cmd::Sequence(command_IntakeObject(Intake).ToPtr(),
                            ArmMovements::ToHighCube(Arm),
                            command_TimeOut([=]{return 1.0;}).ToPtr(),
                            command_OuttakeObject(Intake).ToPtr(),
                            ArmMovements::ToStow(Arm));
}

frc2::CommandPtr autos::ScoreMidCube(subsystem_Intake* Intake, subsystem_Arm* Arm){
  return frc2::cmd::Sequence(command_IntakeObject(Intake).ToPtr(),
                            ArmMovements::ToMidCube(Arm),
                            command_TimeOut([=]{return 1.0;}).ToPtr(),
                            command_OuttakeObject(Intake).ToPtr(),
                            ArmMovements::ToStow(Arm));
}




frc2::CommandPtr autos::OneScoreAndTaxi(subsystem_DriveTrain* DriveTrain, subsystem_PoseTracker* PoseTracker, subsystem_Intake* Intake, subsystem_Arm* Arm){
    return frc2::cmd::Sequence(autos::ScoreHighCube(Intake, Arm), command_DriveAuton(DriveTrain, PoseTracker, "1ST", true, false).ToPtr());
}
frc2::CommandPtr autos::TwoScoreAndTaxi(subsystem_DriveTrain* DriveTrain, subsystem_PoseTracker* PoseTracker, subsystem_Intake* Intake, subsystem_Arm* Arm){
    return frc2::cmd::Sequence(autos::ScoreHighCube(Intake, Arm), command_DriveAuton(DriveTrain, PoseTracker, "2ST", true, false).ToPtr());
}
frc2::CommandPtr autos::ThreeScoreAndTaxi(subsystem_DriveTrain* DriveTrain, subsystem_PoseTracker* PoseTracker, subsystem_Intake* Intake, subsystem_Arm* Arm){
    return frc2::cmd::Sequence(autos::ScoreHighCube(Intake, Arm), command_DriveAuton(DriveTrain, PoseTracker, "3ST", true, false).ToPtr());
}
frc2::CommandPtr autos::ThreeTaxiAndBalance(subsystem_DriveTrain* DriveTrain, subsystem_PoseTracker* PoseTracker){
    return frc2::cmd::Sequence(command_DriveAuton(DriveTrain, PoseTracker, "4TC", true, false).ToPtr(), command_Balance(DriveTrain).ToPtr());
    // return frc2::cmd::Sequence(command_DriveAuton(DriveTrain, PoseTracker, "4TC", true, false).ToPtr(), );

}
frc2::CommandPtr autos::TwoTaxiAndBalance(subsystem_DriveTrain*DriveTrain, subsystem_PoseTracker* PoseTracker){
    return frc2::cmd::Sequence(command_DriveAuton(DriveTrain, PoseTracker, "2TC", true, false).ToPtr(), command_Balance(DriveTrain).ToPtr());
}

frc2::CommandPtr autos::One_ScoreIntakeScore(subsystem_DriveTrain* DriveTrain, subsystem_PoseTracker* PoseTracker, subsystem_Intake* Intake, subsystem_Arm* Arm){
  return frc2::cmd::Sequence(autos::ScoreHighCube(Intake, Arm), 
                            command_DriveAuton(DriveTrain, PoseTracker, "OneToTopPiece", true, false).ToPtr(), 
                            ArmMovements::ToGround(Arm),
                            command_TimeOut([=]{return 2.0;}),
                            frc2::cmd::Race(                            
                            command_DriveAuton(DriveTrain, PoseTracker, "IntakeTopPiece", false, true), 
                            command_IntakeObject(Intake).ToPtr()),
                            ArmMovements::ToStow(Arm), 
                            command_DriveAuton(DriveTrain, PoseTracker, "TopPieceToOne", false, false),
                            autos::ScoreMidCube(Intake, Arm));
}

frc2::CommandPtr autos::Two_ScoreTaxiAndBalance(subsystem_DriveTrain* DriveTrain, subsystem_PoseTracker* PoseTracker, subsystem_Intake* Intake, subsystem_Arm* Arm){
  return frc2::cmd::Sequence(autos::ScoreHighCube(Intake, Arm), 
                              command_DriveAuton(DriveTrain, PoseTracker, "2STC", true, false).ToPtr(), 
                              command_Balance(DriveTrain).ToPtr());
}