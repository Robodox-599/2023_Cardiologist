#include "commands/cGroup_Arm.h"

frc2::CommandPtr ArmMovements::StowToHighCone(subsystem_Arm *Arm) {
  return frc2::cmd::Sequence(command_MoveShoulder(Arm, [=]{return ArmConstants::HighConeShoulder;}, [=]{return false;}).ToPtr());
}
frc2::CommandPtr StowToMidCone(subsystem_Arm *Arm){

}
frc2::CommandPtr StowToHighCube(subsystem_Arm *Arm){

}
frc2::CommandPtr StowToMidCube(subsystem_Arm *Arm){

}
frc2::CommandPtr StowToSubstation(subsystem_Arm *Arm){

}
