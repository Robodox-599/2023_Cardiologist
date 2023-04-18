#include "commands/cGroup_Arm.h"
#include <frc2/command/Commands.h>
#include <frc/smartdashboard/Smartdashboard.h>
#include <frc2/command/WaitCommand.h>

frc2::CommandPtr ArmMovements::GroundTake(subsystem_Arm* Arm, subsystem_GroundTake* GroundTake){
  return frc2::cmd::Sequence( 
                            ToPortal(Arm),
                            frc2::WaitCommand(0.25_s),
                            command_EngageGroundTake(GroundTake).ToPtr(), 
                            GroundTake->WaitUntilRetractedCommand(),
                            TiltedStow(Arm, GroundTake),
                            frc2::WaitCommand(2_s),
                            GroundTake->StopIntakeCommand());
}

frc2::CommandPtr ArmMovements::GroundTakeShoot(subsystem_Arm* Arm, subsystem_GroundTake* GroundTake){
    return frc2::cmd::Sequence(
                          ToPortal(Arm),
                          frc2::WaitCommand(0.5_s),
                          GroundTake->RunHybridIntakeCommand(),
                          frc2::cmd::Race(frc2::WaitCommand(3_s),
                                          GroundTake->WaitUntilEmptyCommand()),
                          TiltedStow(Arm, GroundTake)
    );
}

frc2::CommandPtr ArmMovements::PassThrough(subsystem_Arm* Arm, subsystem_GroundTake* GroundTake, subsystem_Intake* Intake){
return frc2::cmd::Sequence(
    command_MoveElbow(Arm, [=]{return 9.2;}, [=]{return true;}, [=]{return 9.2 * 0.40;}).ToPtr(),
    GroundTake->ExtendGroundTakeCommand(),
    GroundTake->RunPassThroughIntakeCommand(),
    command_MoveWrist(Arm, [=]{return -20.9;}, [=]{return true;}, [=]{return -20.9 * 0.5;}).ToPtr(),
    Intake->UnclampCommand(),
    command_MoveShoulder(Arm, [=]{return 8.5;}, [=]{return true;}).ToPtr(),
    command_MoveElbow(Arm, [=]{return -0.11;}, [=]{return true;}, [=]{return -0.11 * 0.5;}).ToPtr(),
    command_AutoClamp(Intake).ToPtr(),
    command_MoveElbow(Arm, [=]{return 9.2;}, [=]{return true;}, [=]{return 9.2;}).ToPtr(),
    TiltedStow(Arm, GroundTake),
    GroundTake->StopIntakeCommand() 
  );
}





frc2::CommandPtr ArmMovements::ToHighCone(subsystem_Arm *Arm) {
  return frc2::cmd::Sequence( 
                            command_MoveElbow(Arm, [=]{return 4.2;}, [=]{return false;}, [=]{return 4.2;}).ToPtr(),
                            command_MoveWrist(Arm, [=]{return 15;}, [=]{return true;}, [=]{return 15 * 0.25;}).ToPtr(),
                             command_MoveElbow(Arm, [=]{return ArmConstants::HighConeElbow;}, [=]{return true;}, [=]{return ArmConstants::HighConeElbow * 0.80;}).ToPtr(),
                             command_MoveWrist(Arm, [=]{return ArmConstants::HighConeTilt;}, [=]{return false;}, [=]{return ArmConstants::HighConeTilt;}).ToPtr(),
                             command_MoveShoulder(Arm, [=]{return ArmConstants::HighConeShoulder;}, [=]{return false;}).ToPtr());
}

frc2::CommandPtr ArmMovements::ToMidCone(subsystem_Arm *Arm){
  return frc2::cmd::Sequence( 
    command_MoveWrist(Arm, [=]{return 15;}, [=]{return true;}, [=]{return 15 * 0.25;}).ToPtr(),
                              command_MoveElbow(Arm, [=]{return ArmConstants::MidConeElbow;}, [=]{return true;}, [=]{return ArmConstants::MidConeElbow  * 0.50;}).ToPtr(),
                             command_MoveWrist(Arm, [=]{return ArmConstants::MidConeTilt;}, [=]{return false;}, [=]{return ArmConstants::MidConeTilt;}).ToPtr(),
                             command_MoveShoulder(Arm, [=]{return ArmConstants::MidConeShoulder;}, [=]{return false;}).ToPtr());

}

frc2::CommandPtr ArmMovements::ToHighCube(subsystem_Arm *Arm){
  return frc2::cmd::Sequence(
                             command_MoveWrist(Arm, [=]{return 15;}, [=]{return true;}, [=]{return 15 * 0.25;}).ToPtr(),
                             command_MoveElbow(Arm, [=]{return ArmConstants::HighCubeElbow;}, [=]{return true;}, [=]{return ArmConstants::HighCubeElbow * 0.80;}).ToPtr(),
                             command_MoveShoulder(Arm, [=]{return ArmConstants::HighCubeShoulder;}, [=]{return false;}).ToPtr(),
                             command_MoveWrist(Arm, [=]{return ArmConstants::HighCubeTilt;}, [=]{return true;}, [=]{return ArmConstants::HighCubeTilt;}).ToPtr());

}
frc2::CommandPtr ArmMovements::ToMidCube(subsystem_Arm *Arm){
  return frc2::cmd::Sequence(
                             command_MoveWrist(Arm, [=]{return 15;}, [=]{return true;}, [=]{return 15 * 0.25;}).ToPtr(),
                             command_MoveElbow(Arm, [=]{return ArmConstants::MidCubeElbow;}, [=]{return true;}, [=]{return ArmConstants::MidCubeElbow * 0.60;}).ToPtr(),
                             command_MoveShoulder(Arm, [=]{return ArmConstants::MidCubeShoulder;}, [=]{return false;}).ToPtr(),
                            command_MoveWrist(Arm, [=]{return ArmConstants::MidCubeTilt;}, [=]{return true;}, [=]{return ArmConstants::MidCubeTilt * 0.50;}).ToPtr()
);
}

frc2::CommandPtr ArmMovements::ToSubstation(subsystem_Arm *Arm){
  return frc2::cmd::Sequence(command_MoveElbow(Arm, [=]{return ArmConstants::SubstationElbow;}, [=]{return true;}, [=]{return ArmConstants::SubstationElbowThreshold;}).ToPtr(),
                             command_MoveShoulder(Arm, [=]{return ArmConstants::SubstationShoulder;}, [=]{return false;}).ToPtr(),
                             command_MoveWrist(Arm, [=]{return ArmConstants::SubstationTilt;}, [=]{return false;}, [=]{return ArmConstants::SubstationTilt;}).ToPtr());
}

frc2::CommandPtr ArmMovements::ToStow(subsystem_Arm *Arm, subsystem_Intake *Intake){
  return frc2::cmd::Sequence(
                             command_MoveShoulder(Arm, [=]{return ArmConstants::StowShoulder;}, [=]{return false;}).ToPtr(),
                             command_MoveElbow(Arm, [=]{return ArmConstants::StowTempElbow;}, [=]{return true;}, [=]{return ArmConstants::StowTempElbowThreshold;}).ToPtr(),
                             command_MoveWrist(Arm, [=]{return ArmConstants::StowTilt;}, [=]{return true;}, [=]{return ArmConstants::StowTiltThreshold;}).ToPtr(),
                             Intake->ClampCommand(),
                             command_MoveElbow(Arm, [=]{return ArmConstants::StowElbow;}, [=]{return false;}, [=]{return ArmConstants::StowElbow;}).ToPtr());
}

frc2::CommandPtr ArmMovements::ToGround(subsystem_Arm *Arm, subsystem_Intake* Intake){
  return frc2::cmd::Sequence(
                             Intake->UnclampCommand(),
                             command_MoveElbow(Arm, [=]{return ArmConstants::GroundTempElbow;}, [=]{return true;}, [=]{return ArmConstants::GroundTempElbowThreshold;}).ToPtr(),
                             command_MoveWrist(Arm, [=]{return ArmConstants::GroundTilt;}, [=]{return true;}, [=]{return ArmConstants::GroundTiltThreshold;}).ToPtr(),
                             command_MoveElbow(Arm, [=]{return ArmConstants::GroundElbow;}, [=]{return false;}, [=]{return ArmConstants::GroundElbow;}).ToPtr(),
                             command_MoveShoulder(Arm, [=]{return ArmConstants::GroundShoulder;}, [=]{return false;}).ToPtr());
}

frc2::CommandPtr ArmMovements::ToPortal(subsystem_Arm *Arm){
  return frc2::cmd::Sequence(
                             command_MoveElbow(Arm, [=]{return 10;}, [=]{return true;}, [=]{return 10 * 0.5;}).ToPtr(),
                             command_MoveWrist(Arm, [=]{return ArmConstants::PortalTilt;}, [=]{return true;}, [=]{return ArmConstants::PortalTiltThreshold;}).ToPtr(),
                             command_MoveShoulder(Arm, [=]{return ArmConstants::StowShoulder;}, [=]{return true;}).ToPtr(),
                             command_MoveElbow(Arm, [=]{return ArmConstants::PortalElbow;}, [=]{return false;}, [=]{return ArmConstants::PortalElbow;}).ToPtr());
}

frc2::CommandPtr ArmMovements::ToPortalAndIntake(subsystem_Arm *Arm, subsystem_Intake *Intake){
  return frc2::cmd::Sequence(
                              ArmMovements::ToPortal(Arm),
                              command_AutoClamp(Intake).ToPtr());
}

frc2::CommandPtr ArmMovements::ScoreCubeAndStow(subsystem_Arm *Arm, subsystem_Intake *Intake){
    return frc2::cmd::Sequence(
                            Intake->UnclampCommand(),
                             command_MoveShoulder(Arm, [=]{return ArmConstants::StowShoulder;}, [=]{return false;}).ToPtr(),
                             command_MoveElbow(Arm, [=]{return ArmConstants::StowTempElbow;}, [=]{return true;}, [=]{return ArmConstants::StowTempElbow;}).ToPtr(),
                             command_MoveWrist(Arm, [=]{return ArmConstants::StowTilt;}, [=]{return true;}, [=]{return ArmConstants::StowTiltThreshold;}).ToPtr(),
                            Intake->ClampCommand(),
                             command_MoveElbow(Arm, [=]{return ArmConstants::StowCubeElbow;}, [=]{return false;}, [=]{return ArmConstants::StowCubeElbow;}).ToPtr());
}

frc2::CommandPtr ArmMovements::ScoreConeAndStow(subsystem_Arm *Arm, subsystem_Intake *Intake){
    return frc2::cmd::Sequence(
                              command_MoveWrist(Arm, [=]{return ArmConstants::ScoreTilt;}, [=]{return true;}, [=]{return ArmConstants::ScoreTilt;}).ToPtr(),
                              command_TimeOut([=]{return ArmConstants::ScoreTimeout;}).ToPtr(),
                              Intake->UnclampCommand(),
                              ArmMovements::ToStow(Arm, Intake));
}

frc2::CommandPtr ArmMovements::StowFromMidCube(subsystem_Arm *Arm, subsystem_Intake *Intake){
    return frc2::cmd::Sequence(
                             Intake->UnclampCommand(),
                             command_TimeOut([=]{return ArmConstants::ScoreTimeout;}).ToPtr(),
                             command_MoveShoulder(Arm, [=]{return ArmConstants::StowShoulder;}, [=]{return true;}).ToPtr(),
                             Intake->ClampCommand(),
                             command_MoveWrist(Arm, [=]{return ArmConstants::StowTilt;}, [=]{return true;}, [=]{return ArmConstants::StowMidCubeTiltThreshold;}).ToPtr(),
                             command_MoveElbow(Arm, [=]{return ArmConstants::StowTempElbow;}, [=]{return false;}, [=]{return ArmConstants::StowTempElbow;}).ToPtr(),
                             command_MoveElbow(Arm, [=]{return ArmConstants::StowCubeElbow;}, [=]{return false;}, [=]{return ArmConstants::StowCubeElbow;}).ToPtr());
}

frc2::CommandPtr ArmMovements::StowFromHighCube(subsystem_Arm *Arm, subsystem_Intake *Intake){
    return frc2::cmd::Sequence(
                            Intake->UnclampCommand(),
                              command_TimeOut([=]{return ArmConstants::ScoreTimeout;}).ToPtr(),
                             command_MoveShoulder(Arm, [=]{return ArmConstants::StowShoulder;}, [=]{return true;}).ToPtr(),
                            Intake->ClampCommand(),
                             command_MoveWrist(Arm, [=]{return ArmConstants::StowTilt;}, [=]{return true;}, [=]{return ArmConstants::StowHighCubeTiltThreshold;}).ToPtr(),
                             command_MoveElbow(Arm, [=]{return ArmConstants::StowTempElbow;}, [=]{return false;}, [=]{return ArmConstants::StowTempElbow;}).ToPtr(),
                             command_MoveElbow(Arm, [=]{return ArmConstants::StowCubeElbow;}, [=]{return false;}, [=]{return ArmConstants::StowCubeElbow;}).ToPtr());
}

frc2::CommandPtr ArmMovements::HighConeScoreAndStow(subsystem_Arm *Arm, subsystem_Intake *Intake){

    return frc2::cmd::Sequence( ArmMovements::ToHighCone(Arm), 
                                command_TimeOut([=]{return 1.0;}).ToPtr(),
                                ArmMovements::ScoreConeAndStow(Arm, Intake) );
}

frc2::CommandPtr ArmMovements::MidConeScoreAndStow(subsystem_Arm *Arm, subsystem_Intake *Intake){
    return frc2::cmd::Sequence( ArmMovements::ToMidCone(Arm),
                                command_TimeOut([=]{return ArmConstants::ScoreTimeout;}).ToPtr(),
                                ArmMovements::ScoreConeAndStow(Arm,Intake) );
}

frc2::CommandPtr ArmMovements::HighCubeScoreAndStow(subsystem_Arm *Arm, subsystem_Intake *Intake){
    return frc2::cmd::Sequence( ArmMovements::ToHighCube(Arm),
                                command_TimeOut([=]{return ArmConstants::ScoreTimeout;}).ToPtr(),
                                ArmMovements::StowFromHighCube(Arm, Intake));
}

frc2::CommandPtr ArmMovements::MidCubeScoreAndStow(subsystem_Arm *Arm, subsystem_Intake *Intake){
    return frc2::cmd::Sequence( ArmMovements::ToMidCube(Arm),
                                command_TimeOut([=]{return ArmConstants::ScoreTimeout;}).ToPtr(),
                                ArmMovements::StowFromMidCube(Arm, Intake));
}

frc2::CommandPtr ArmMovements::HybridScoreAndStow(subsystem_Arm *Arm, subsystem_Intake *Intake){
  return frc2::cmd::Sequence( ArmMovements::ToPortal(Arm),
                              command_TimeOut([=]{return ArmConstants::ScoreTimeout;}).ToPtr(),
                              Intake->UnclampCommand(),
                              ArmMovements::ToStow(Arm, Intake)
                              );
}

frc2::CommandPtr ArmMovements::TiltedStow(subsystem_Arm* Arm, subsystem_GroundTake* GroundTake){
  return frc2::cmd::Sequence(
                              GroundTake->StowCompleteleyCommand(),
                              // GroundTake->WaitUntilRetractedCommand(),
                              command_MoveWrist(Arm, [=]{return 14.0;}, [=]{return false;}, [=]{return 14.0 * 0.25;}),
                              command_MoveShoulder(Arm, [=]{return ArmConstants::TiltedStowShoulder;}, [=]{return true;}).ToPtr(),
                              command_MoveElbow(Arm, [=]{return ArmConstants::TiltedStowElbow;}, [=]{return false;}, [=]{return ArmConstants::TiltedStowElbow;}).ToPtr(),
                              command_MoveWrist(Arm, [=]{return ArmConstants::ScoreTilt;}, [=]{return false;}, [=]{return ArmConstants::TiltedStowTilt;}).ToPtr());
}

// frc2::CommandPtr ArmMovements::ConeScore(subsystem_Arm* Arm, subsystem_Intake* Intake, std::function<int()> NODE_LEVEL){
//     DPAD::NODE_LEVEL ArmPoll = DPAD::NODE_LEVEL::NON_SPECIFIED;
//     frc::SmartDashboard::PutNumber("Node Level", NODE_LEVEL());
//     // // frc::Smartdashboard::PutNumber("ArmPoll", ArmPoll);
//     // frc::SmartDashboard::PutNumber("arm poll", ArmPoll);
//     // Arm->PollArmPosition(NODE_LEVEL());

//     switch(NODE_LEVEL()){
//         case 0:
//             ArmPoll = DPAD::NODE_LEVEL::HIGH;
//             break;
//         case 270:
//             ArmPoll = DPAD::NODE_LEVEL::MID;
//             break;
//         case 180:
//             ArmPoll = DPAD::NODE_LEVEL::LOW;
//             break;
//         default:
//             ArmPoll = DPAD::NODE_LEVEL::NON_SPECIFIED;
//             break;
//     }
//     frc::SmartDashboard::PutNumber("ArmPoll", NODE_LEVEL());

//     switch(ArmPoll){
//         case DPAD::NODE_LEVEL::HIGH:
//           return ArmMovements::HighConeScoreAndStow(Arm, Intake);
//           break;
//         case DPAD::NODE_LEVEL::MID:
//           return ArmMovements::MidConeScoreAndStow(Arm, Intake);
//           break;
//         case DPAD::NODE_LEVEL::LOW:
//           return ArmMovements::HybridScoreAndStow(Arm, Intake);
//           break;
//         case::DPAD::NODE_LEVEL::NON_SPECIFIED:
//           return command_TimeOut([=]{return 0.0;}).ToPtr();
//           break;
//     }

// }

// frc2::CommandPtr ArmMovements::CubeScore(subsystem_Arm* Arm, subsystem_Intake* Intake){
//     DPAD::NODE_LEVEL ArmPoll = Arm->GetArmPoll();

//     switch(ArmPoll){
//         case DPAD::NODE_LEVEL::HIGH:
//           return ArmMovements::HighCubeScoreAndStow(Arm, Intake);
//         case DPAD::NODE_LEVEL::MID:
//           return ArmMovements::MidCubeScoreAndStow(Arm, Intake);
//         case DPAD::NODE_LEVEL::LOW:
//           return ArmMovements::HybridScoreAndStow(Arm, Intake);
//         case::DPAD::NODE_LEVEL::NON_SPECIFIED:
//           return command_TimeOut([=]{return 0.0;}).ToPtr();
//     }
// }

// frc2::CommandPtr ArmMovements::ToCone(subsystem_Arm* Arm, subsystem_Intake* Intake){
//     DPAD::NODE_LEVEL ArmPoll = Arm->GetArmPoll();

//     switch(ArmPoll){
//         case DPAD::NODE_LEVEL::HIGH:
//           return ArmMovements::ToHighCone(Arm);
//         case DPAD::NODE_LEVEL::MID:
//           return ArmMovements::ToMidCone(Arm);
//         case DPAD::NODE_LEVEL::LOW:
//           return ArmMovements::ToFloorScore(Arm);
//         case::DPAD::NODE_LEVEL::NON_SPECIFIED:
//           return command_TimeOut([=]{return 0.0;}).ToPtr();
//     }

// }

// frc2::CommandPtr ArmMovements::ToCube(subsystem_Arm* Arm, subsystem_Intake* Intake){
//     DPAD::NODE_LEVEL ArmPoll = Arm->GetArmPoll();
//     switch(ArmPoll){
//         case DPAD::NODE_LEVEL::HIGH:
//           return ArmMovements::ToHighCube(Arm);
//         case DPAD::NODE_LEVEL::MID:
//           return ArmMovements::ToMidCube(Arm);
//         case DPAD::NODE_LEVEL::LOW:
//           return ArmMovements::ToFloorScore(Arm);
//         case::DPAD::NODE_LEVEL::NON_SPECIFIED:
//           return command_TimeOut([=]{return 0.0;}).ToPtr();
//     }
// }



// frc2::CommandPtr ArmMovements::ToMidCone(subsystem_Arm *Arm){
//   return frc2::cmd::Sequence(
//                              command_MoveElbow(Arm, [=]{return ArmConstants::MidConeElbow * 0.25;}, [=]{return true;}, [=]{return ArmConstants::MidConeElbow * 0.25;}).ToPtr(),
//                              command_MoveWrist(Arm, [=]{return ArmConstants::MidConeTilt;}, [=]{return true;}, [=]{return ArmConstants::MidConeTilt;}).ToPtr(),
//                              command_MoveElbow(Arm, [=]{return ArmConstants::MidConeElbow;}, [=]{return true;}, [=]{return ArmConstants::MidConeElbow * 0.80;}).ToPtr(),
//                              command_MoveShoulder(Arm, [=]{return ArmConstants::MidConeShoulder;}, [=]{return false;}).ToPtr());

//     // return frc2::cmd::Sequence(
//     //                          command_MoveElbow(Arm, [=]{return ArmConstants::MidConeElbow;}, [=]{return true;}, [=]{return ArmConstants::MidConeElbow * 0.80 ;}).ToPtr(),
//     //                          command_MoveShoulder(Arm, [=]{return ArmConstants::MidConeShoulder;}, [=]{return false;}).ToPtr(),
//     //                          command_MoveWrist(Arm, [=]{return ArmConstants::MidConeTilt;}, [=]{return false;}, [=]{return 1000;}).ToPtr());

// }
// frc2::CommandPtr ArmMovements::ToHighCube(subsystem_Arm *Arm){
//   return frc2::cmd::Sequence(
//                              command_MoveWrist(Arm, [=]{return ArmConstants::HighCubeTilt;}, [=]{return true;}, [=]{return ArmConstants::HighCubeTilt * 0.50;}).ToPtr(),
//                              command_MoveElbow(Arm, [=]{return ArmConstants::HighCubeElbow;}, [=]{return true;}, [=]{return ArmConstants::HighCubeElbow * 0.80;}).ToPtr(),
//                              command_MoveShoulder(Arm, [=]{return ArmConstants::HighCubeShoulder;}, [=]{return false;}).ToPtr());

// }
// frc2::CommandPtr ArmMovements::ToMidCube(subsystem_Arm *Arm){
//   return frc2::cmd::Sequence(
//                              command_MoveWrist(Arm, [=]{return ArmConstants::MidCubeTilt;}, [=]{return true;}, [=]{return ArmConstants::MidCubeTilt * 0.50;}).ToPtr(),
//                              command_MoveElbow(Arm, [=]{return ArmConstants::MidCubeElbow;}, [=]{return true;}, [=]{return ArmConstants::MidCubeElbow * 0.80;}).ToPtr(),
//                              command_MoveShoulder(Arm, [=]{return ArmConstants::MidCubeShoulder;}, [=]{return false;}).ToPtr());
// }

// frc2::CommandPtr ArmMovements::ToFloorScore(subsystem_Arm *Arm){
//   return frc2::cmd::Sequence(
//                              command_MoveShoulder(Arm, [=]{return ArmConstants::StowShoulder;}, [=]{return false;}).ToPtr(),
//                              command_MoveElbow(Arm, [=]{return ArmConstants::StowElbow;}, [=]{return false;}, [=]{return 100000;}).ToPtr(),
//                              command_MoveWrist(Arm, [=]{return ArmConstants::floorCubeTilt;}, [=]{return false;}, [=]{return 1000;}).ToPtr());
// }

