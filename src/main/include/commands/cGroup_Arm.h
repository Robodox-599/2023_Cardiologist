#pragma once

#include <frc2/command/CommandPtr.h>
#include "subsystems/subsystem_Arm.h"
#include "commands/command_MoveElbow.h"
#include "commands/command_MoveShoulder.h"
#include "commands/command_MoveWrist.h"
#include "Constants.h"
#include "subsystems/ExampleSubsystem.h"

namespace ArmMovements
{
  frc2::CommandPtr StowToHighCone(subsystem_Arm *Arm);
  frc2::CommandPtr StowToMidCone(subsystem_Arm *Arm);
  frc2::CommandPtr StowToHighCube(subsystem_Arm *Arm);
  frc2::CommandPtr StowToMidCube(subsystem_Arm *Arm);
  frc2::CommandPtr StowToSubstation(subsystem_Arm *Arm);
  frc2::CommandPtr ToStow(subsystem_Arm *Arm);

}