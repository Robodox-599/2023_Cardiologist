// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/Autos.h"

#include <frc2/command/Commands.h>

#include "commands/ExampleCommand.h"



frc2::CommandPtr autos::TestAuto(subsystem_DriveTrain* DriveTrain, subsystem_PoseTracker* PoseTracker){
  return command_DriveAuton(DriveTrain, PoseTracker, "Test",  true).ToPtr();
}

/*
frc2::CommandPtr autos::Niemann(subsystem_DriveTrain* DriveTrain, subsystem_PoseTracker* PoseTracker){
  return frc2::cmd::Sequence( command_DriveAuton(DriveTrain, PoseTracker, "Niemann (Yes^2)", true).ToPtr(), 
                              command_DriveAuton(DriveTrain, PoseTracker, "Niemann 2 (Yes^2)", false).ToPtr(),
                              command_DriveAuton(DriveTrain, PoseTracker, "Niemann 3 (Yes^2)", false).ToPtr());
}
*/
frc2::CommandPtr autos::Kasparov(subsystem_DriveTrain* DriveTrain, subsystem_PoseTracker* PoseTracker){
  return frc2::cmd::Sequence( command_DriveAuton(DriveTrain, PoseTracker, "Kasparov (4T)", true).ToPtr() );
}


frc2::CommandPtr autos::ScoreHighCube(subsystem_Intake* Intake, subsystem_Arm* Arm){
  return frc2::cmd::Sequence(command_IntakeObject(Intake).ToPtr(),
                            ArmMovements::ToHighCube(Arm),
                            command_TimeOut([=]{return 1.0;}).ToPtr(),
                            command_OuttakeObject(Intake).ToPtr(),
                            ArmMovements::ToStow(Arm));
}
frc2::CommandPtr autos::OneSTIS_1(subsystem_DriveTrain* DriveTrain, subsystem_PoseTracker* PoseTracker, subsystem_Intake* Intake, subsystem_Arm* Arm){
  return frc2::cmd::Parallel(ArmMovements::ToGround(Arm),
                            command_IntakeObject(Intake).ToPtr(),
                            command_DriveAuton(DriveTrain, PoseTracker, "1STIS (1)", true).ToPtr());
}
frc2::CommandPtr autos::OneSTIS_2(subsystem_DriveTrain* DriveTrain, subsystem_PoseTracker* PoseTracker){
  return frc2::cmd::Parallel(command_DriveAuton(DriveTrain, PoseTracker, "1STIS (2)", true).ToPtr());
}
frc2::CommandPtr autos::OneSTIS_3(subsystem_Intake* Intake, subsystem_Arm* Arm){
  return frc2::cmd::Sequence(ArmMovements::ToHighCone(Arm),
                            command_OuttakeObject(Intake).ToPtr());
}

frc2::CommandPtr autos::TwoSISC_0(subsystem_Intake* Intake, subsystem_Arm* Arm){
  return frc2::cmd::Sequence(ArmMovements::ToHighCube(Arm),
                            command_OuttakeObject(Intake).ToPtr(),
                            ArmMovements::ToStow(Arm));
}
frc2::CommandPtr autos::TwoSISC_1(subsystem_DriveTrain* DriveTrain, subsystem_PoseTracker* PoseTracker, subsystem_Intake* Intake, subsystem_Arm* Arm){
  return frc2::cmd::Parallel(ArmMovements::ToGround(Arm),
                            command_IntakeObject(Intake).ToPtr(),
                            command_DriveAuton(DriveTrain, PoseTracker, "2SISC (1)", true).ToPtr());
}
frc2::CommandPtr autos::TwoSISC_2(subsystem_DriveTrain* DriveTrain, subsystem_PoseTracker* PoseTracker){
  return frc2::cmd::Parallel(command_DriveAuton(DriveTrain, PoseTracker, "2SISC (2)", true).ToPtr());
}
frc2::CommandPtr autos::TwoSISC_2_half(subsystem_Intake* Intake, subsystem_Arm* Arm){
  return frc2::cmd::Sequence(ArmMovements::ToHighCone(Arm),
                            command_OuttakeObject(Intake).ToPtr());
}
frc2::CommandPtr autos::TwoSISC_3(subsystem_DriveTrain* DriveTrain, subsystem_PoseTracker* PoseTracker, subsystem_Arm* Arm){
  return frc2::cmd::Parallel(ArmMovements::ToStow(Arm),
                            command_DriveAuton(DriveTrain, PoseTracker, "2SISC (3)", true).ToPtr());
}

frc2::CommandPtr autos::ThreeSISIS_0(subsystem_Intake* Intake, subsystem_Arm* Arm){
  return frc2::cmd::Sequence(ArmMovements::ToHighCube(Arm),
                            command_OuttakeObject(Intake).ToPtr(),
                            ArmMovements::ToStow(Arm));
}

frc2::CommandPtr autos::ThreeSISIS_1(subsystem_DriveTrain* DriveTrain, subsystem_PoseTracker* PoseTracker, subsystem_Intake* Intake, subsystem_Arm* Arm){
  return frc2::cmd::Parallel(ArmMovements::ToGround(Arm),
                            command_IntakeObject(Intake).ToPtr(),
                            command_DriveAuton(DriveTrain, PoseTracker, "ThreeSISIS (1)", true).ToPtr());
}

frc2::CommandPtr autos::ThreeSISIS_2(subsystem_DriveTrain* DriveTrain, subsystem_PoseTracker* PoseTracker){
  return frc2::cmd::Parallel(command_DriveAuton(DriveTrain, PoseTracker, "ThreeSISIS (2)", true).ToPtr());
}

frc2::CommandPtr autos::ThreeSISIS_2_half(subsystem_Intake* Intake, subsystem_Arm* Arm){
  return frc2::cmd::Sequence(ArmMovements::ToHighCone(Arm),
                            command_OuttakeObject(Intake).ToPtr(),
                            ArmMovements::ToStow(Arm));
}

frc2::CommandPtr autos::ThreeSISIS_3(subsystem_DriveTrain* DriveTrain, subsystem_PoseTracker* PoseTracker, subsystem_Intake* Intake, subsystem_Arm* Arm){
  return frc2::cmd::Parallel(ArmMovements::ToGround(Arm),
                            command_IntakeObject(Intake).ToPtr(),
                            command_DriveAuton(DriveTrain, PoseTracker, "ThreeSISIS (3)", true).ToPtr());
}

frc2::CommandPtr autos::ThreeSISIS_4(subsystem_DriveTrain* DriveTrain, subsystem_PoseTracker* PoseTracker){
  return frc2::cmd::Parallel(command_DriveAuton(DriveTrain, PoseTracker, "ThreeSISIS (4)", true).ToPtr());
}

frc2::CommandPtr autos::ThreeSISIS_5(subsystem_Intake* Intake, subsystem_Arm* Arm){
  return frc2::cmd::Sequence(ArmMovements::ToHighCone(Arm),
                            command_OuttakeObject(Intake).ToPtr());
}


frc2::CommandPtr autos::ThreeSTIS_0(subsystem_Intake* Intake, subsystem_Arm* Arm){
  return frc2::cmd::Sequence(ArmMovements::ToHighCube(Arm),
                            command_OuttakeObject(Intake).ToPtr(),
                            ArmMovements::ToStow(Arm));
}

frc2::CommandPtr autos::ThreeSTIS_1(subsystem_DriveTrain* DriveTrain, subsystem_PoseTracker* PoseTracker, subsystem_Intake* Intake, subsystem_Arm* Arm){
  return frc2::cmd::Parallel(ArmMovements::ToGround(Arm),
                            command_IntakeObject(Intake).ToPtr(),
                            command_DriveAuton(DriveTrain, PoseTracker, "ThreeSTIS (1)", true).ToPtr());
}

frc2::CommandPtr autos::ThreeSTIS_2(subsystem_DriveTrain* DriveTrain, subsystem_PoseTracker* PoseTracker){
  return frc2::cmd::Parallel(command_DriveAuton(DriveTrain, PoseTracker, "ThreeSTIS (2)", true).ToPtr());
}

frc2::CommandPtr autos::ThreeSTIS_3(subsystem_Intake* Intake, subsystem_Arm* Arm){
  return frc2::cmd::Sequence(ArmMovements::ToHighCone(Arm),
                            command_OuttakeObject(Intake).ToPtr());
}

frc2::CommandPtr autos::TwoSTISC_0(subsystem_Intake* Intake, subsystem_Arm* Arm){
  return frc2::cmd::Sequence(ArmMovements::ToHighCube(Arm),
                            command_OuttakeObject(Intake).ToPtr(),
                            ArmMovements::ToStow(Arm));
}

frc2::CommandPtr autos::TwoSTISC_1(subsystem_DriveTrain* DriveTrain, subsystem_PoseTracker* PoseTracker, subsystem_Intake* Intake, subsystem_Arm* Arm){
  return frc2::cmd::Parallel(ArmMovements::ToGround(Arm),
                            command_IntakeObject(Intake).ToPtr(),
                            command_DriveAuton(DriveTrain, PoseTracker, "TwoSTISC (1)", true).ToPtr());
}

frc2::CommandPtr autos::TwoSTISC_2(subsystem_DriveTrain* DriveTrain, subsystem_PoseTracker* PoseTracker){
  return frc2::cmd::Parallel(command_DriveAuton(DriveTrain, PoseTracker, "TwoSTISC (2)", true).ToPtr());
}

frc2::CommandPtr autos::TwoSTISC_3(subsystem_DriveTrain* DriveTrain, subsystem_PoseTracker* PoseTracker, subsystem_Intake* Intake, subsystem_Arm* Arm){
  return frc2::cmd::Sequence(ArmMovements::ToHighCone(Arm),
                            command_OuttakeObject(Intake).ToPtr(),
                            command_DriveAuton(DriveTrain, PoseTracker, "TwoSTISC (3)", true).ToPtr());
}

frc2::CommandPtr autos::OneSISIS_0(subsystem_Intake* Intake, subsystem_Arm* Arm){
  return frc2::cmd::Sequence(ArmMovements::ToHighCube(Arm),
                            command_OuttakeObject(Intake).ToPtr(),
                            ArmMovements::ToStow(Arm));   
}

frc2::CommandPtr autos::OneSISIS_1(subsystem_DriveTrain* DriveTrain, subsystem_PoseTracker* PoseTracker, subsystem_Intake* Intake, subsystem_Arm* Arm){
  return frc2::cmd::Parallel(ArmMovements::ToGround(Arm),
                            command_IntakeObject(Intake).ToPtr(),
                            command_DriveAuton(DriveTrain, PoseTracker, "Yes^3 (1)", true).ToPtr());
}

frc2::CommandPtr autos::OneSISIS_2(subsystem_DriveTrain* DriveTrain, subsystem_PoseTracker* PoseTracker){
  return frc2::cmd::Parallel(command_DriveAuton(DriveTrain, PoseTracker, "Yes^3 (2)", true).ToPtr());
}

frc2::CommandPtr autos::OneSISIS_2_half(subsystem_Intake* Intake, subsystem_Arm* Arm){
  return frc2::cmd::Sequence(ArmMovements::ToHighCone(Arm),
                            command_OuttakeObject(Intake).ToPtr()); 
}

frc2::CommandPtr autos::OneSISIS_3(subsystem_DriveTrain* DriveTrain, subsystem_PoseTracker* PoseTracker, subsystem_Intake* Intake, subsystem_Arm* Arm){
  return frc2::cmd::Parallel(ArmMovements::ToGround(Arm),
                            command_IntakeObject(Intake).ToPtr(),
                            command_DriveAuton(DriveTrain, PoseTracker, "Yes^3 (3)", true).ToPtr());
}

frc2::CommandPtr autos::OneSISIS_4(subsystem_DriveTrain* DriveTrain, subsystem_PoseTracker* PoseTracker){
  return frc2::cmd::Parallel(command_DriveAuton(DriveTrain, PoseTracker, "Yes^3 (4)", true).ToPtr());
}

frc2::CommandPtr autos::OneSISIS_5(subsystem_Intake* Intake, subsystem_Arm* Arm){
  return frc2::cmd::Sequence(ArmMovements::ToHighCone(Arm), 
                            command_OuttakeObject(Intake).ToPtr());
}

frc2::CommandPtr autos::OneSTIS(subsystem_DriveTrain* DriveTrain, subsystem_PoseTracker* PoseTracker, subsystem_Intake* Intake, subsystem_Arm* Arm){
  return frc2::cmd::Sequence(autos::ScoreHighCube(Intake, Arm),
                            autos::OneSTIS_1(DriveTrain, PoseTracker, Intake, Arm),
                            autos::OneSTIS_2(DriveTrain, PoseTracker),
                            autos::OneSTIS_3(Intake, Arm)); 
}

frc2::CommandPtr autos::TwoSISC(subsystem_DriveTrain* DriveTrain, subsystem_PoseTracker* PoseTracker, subsystem_Intake* Intake, subsystem_Arm* Arm){
  return frc2::cmd::Sequence(autos::TwoSISC_0(Intake, Arm),
                            autos::TwoSISC_1(DriveTrain, PoseTracker, Intake, Arm),
                            autos::TwoSISC_2(DriveTrain, PoseTracker),
                            autos::TwoSISC_2_half(Intake, Arm),
                            autos::TwoSISC_3(DriveTrain, PoseTracker, Arm));  
}

frc2::CommandPtr autos::ThreeSISIS(subsystem_DriveTrain* DriveTrain, subsystem_PoseTracker* PoseTracker, subsystem_Intake* Intake, subsystem_Arm* Arm){
  return frc2::cmd::Sequence(autos::ThreeSISIS_0(Intake, Arm),
                            autos::ThreeSISIS_1(DriveTrain, PoseTracker, Intake, Arm),
                            autos::ThreeSISIS_2(DriveTrain, PoseTracker),
                            autos::ThreeSISIS_2_half(Intake, Arm),
                            autos::ThreeSISIS_3(DriveTrain, PoseTracker, Intake, Arm),
                            autos::ThreeSISIS_4(DriveTrain, PoseTracker),
                            autos::ThreeSISIS_5(Intake, Arm));  
}

frc2::CommandPtr autos::ThreeSTIS(subsystem_DriveTrain* DriveTrain, subsystem_PoseTracker* PoseTracker, subsystem_Intake* Intake, subsystem_Arm* Arm){
  return frc2::cmd::Sequence(autos::ThreeSTIS_0(Intake, Arm),
    autos::ThreeSTIS_1(DriveTrain, PoseTracker, Intake, Arm),
    autos::ThreeSTIS_2(DriveTrain, PoseTracker),
    autos::ThreeSTIS_3(Intake, Arm));  
}

frc2::CommandPtr autos::TwoSTISC(subsystem_DriveTrain* DriveTrain, subsystem_PoseTracker* PoseTracker, subsystem_Intake* Intake, subsystem_Arm* Arm){
  return frc2::cmd::Sequence(autos::TwoSTISC_0(Intake, Arm),
                            autos::TwoSTISC_1(DriveTrain, PoseTracker, Intake, Arm),
                            autos::TwoSTISC_2(DriveTrain, PoseTracker),
                            autos::TwoSTISC_3(DriveTrain, PoseTracker, Intake, Arm));  
}

frc2::CommandPtr autos::OneSISIS(subsystem_DriveTrain* DriveTrain, subsystem_PoseTracker* PoseTracker, subsystem_Intake* Intake, subsystem_Arm* Arm){
  return frc2::cmd::Sequence(autos::OneSISIS_0(Intake, Arm),
                            autos::OneSISIS_1(DriveTrain, PoseTracker, Intake, Arm),
                            autos::OneSISIS_2(DriveTrain, PoseTracker),
                            autos::OneSISIS_2_half(Intake, Arm),
                            autos::OneSISIS_3(DriveTrain, PoseTracker, Intake, Arm),
                            autos::OneSISIS_4(DriveTrain, PoseTracker),
                            autos::OneSISIS_5(Intake, Arm));  

}

frc2::CommandPtr autos::OneScoreAndTaxi(subsystem_DriveTrain* DriveTrain, subsystem_PoseTracker* PoseTracker, subsystem_Intake* Intake, subsystem_Arm* Arm){
    return frc2::cmd::Sequence(autos::ScoreHighCube(Intake, Arm), command_DriveAuton(DriveTrain, PoseTracker, "1ST", true).ToPtr());
}
frc2::CommandPtr autos::TwoScoreAndTaxi(subsystem_DriveTrain* DriveTrain, subsystem_PoseTracker* PoseTracker, subsystem_Intake* Intake, subsystem_Arm* Arm){
    return frc2::cmd::Sequence(autos::ScoreHighCube(Intake, Arm), command_DriveAuton(DriveTrain, PoseTracker, "2ST", true).ToPtr());
}
frc2::CommandPtr autos::ThreeScoreAndTaxi(subsystem_DriveTrain* DriveTrain, subsystem_PoseTracker* PoseTracker, subsystem_Intake* Intake, subsystem_Arm* Arm){
    return frc2::cmd::Sequence(autos::ScoreHighCube(Intake, Arm), command_DriveAuton(DriveTrain, PoseTracker, "3ST", true).ToPtr());
}

frc2::CommandPtr autos::ThreeTaxiAndBalance(subsystem_DriveTrain* DriveTrain, subsystem_PoseTracker* PoseTracker){
    return frc2::cmd::Sequence(command_DriveAuton(DriveTrain, PoseTracker, "4TC", true).ToPtr(), command_Balance(DriveTrain).ToPtr());
    // return frc2::cmd::Sequence(command_DriveAuton(DriveTrain, PoseTracker, "4TC", true).ToPtr(), );

}

frc2::CommandPtr autos::TwoTaxiAndBalance(subsystem_DriveTrain*DriveTrain, subsystem_PoseTracker* PoseTracker){
    return frc2::cmd::Sequence(command_DriveAuton(DriveTrain, PoseTracker, "2TC", true).ToPtr(), command_Balance(DriveTrain).ToPtr());
}