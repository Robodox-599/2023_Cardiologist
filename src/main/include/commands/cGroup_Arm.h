#pragma once

#include <frc2/command/CommandPtr.h>
#include "subsystems/subsystem_Arm.h"
#include "subsystems/subsystem_GroundTake.h"
#include "commands/command_MoveElbow.h"
#include "commands/command_MoveShoulder.h"
#include "commands/command_MoveWrist.h"
#include "commands/command_AutoClamp.h"
#include "commands/command_Clamp.h"
#include "commands/command_TimeOut.h"
#include "commands/command_EngageGroundTake.h"
#include <frc2/command/WaitCommand.h>
#include "Constants.h"

namespace ArmMovements
{
  frc2::CommandPtr ToHighCone(subsystem_Arm *Arm);
  frc2::CommandPtr ToMidCone(subsystem_Arm *Arm);
  frc2::CommandPtr ToHighCube(subsystem_Arm *Arm);
  frc2::CommandPtr ToMidCube(subsystem_Arm *Arm);
  frc2::CommandPtr ToSubstation(subsystem_Arm *Arm);
  frc2::CommandPtr ToFloorScore(subsystem_Arm *Arm);
  frc2::CommandPtr ToGround(subsystem_Arm *Arm);
  frc2::CommandPtr ToStow(subsystem_Arm *Arm, subsystem_Intake* Intake);

  frc2::CommandPtr ToPortal(subsystem_Arm *Arm);
  frc2::CommandPtr ToPortalAndIntake(subsystem_Arm *Arm, subsystem_Intake *Intake);


  frc2::CommandPtr ScoreAndStow(subsystem_Arm *Arm, subsystem_Intake* Intake);
  frc2::CommandPtr ScoreConeAndStow(subsystem_Arm *Arm, subsystem_Intake* Intake);

  frc2::CommandPtr HighConeScoreAndStow(subsystem_Arm *Arm, subsystem_Intake* Intake);
  frc2::CommandPtr MidConeScoreAndStow(subsystem_Arm* Arm, subsystem_Intake* Intake);
  frc2::CommandPtr HighCubeScoreAndStow(subsystem_Arm* Arm, subsystem_Intake* Intake);
  frc2::CommandPtr MidCubeScoreAndStow(subsystem_Arm* Arm, subsystem_Intake* Intake);

  frc2::CommandPtr HybridScoreAndStow(subsystem_Arm* Arm, subsystem_Intake* Intake);

  frc2::CommandPtr CubeScore(subsystem_Arm* Arm, subsystem_Intake* Intake);
  frc2::CommandPtr ConeScore(subsystem_Arm* Arm, subsystem_Intake* Intake);

  frc2::CommandPtr ToCube(subsystem_Arm* Arm, subsystem_Intake* Intake);
  frc2::CommandPtr ToCone(subsystem_Arm* Arm, subsystem_Intake* Intake);

  frc2::CommandPtr GroundTakeTest(subsystem_GroundTake* GroundTake);
}