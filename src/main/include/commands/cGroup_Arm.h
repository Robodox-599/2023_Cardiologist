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
  frc2::CommandPtr ToHighCone(subsystem_Arm *Arm);
  frc2::CommandPtr ToMidCone(subsystem_Arm *Arm);
  frc2::CommandPtr ToHighCube(subsystem_Arm *Arm);
  frc2::CommandPtr ToMidCube(subsystem_Arm *Arm);
  frc2::CommandPtr ToSubstation(subsystem_Arm *Arm);
  frc2::CommandPtr ToStow(subsystem_Arm *Arm);
  frc2::CommandPtr ToGround(subsystem_Arm *Arm);

}