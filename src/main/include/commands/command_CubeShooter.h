// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include "subsystems/subsystem_Arm.h"
#include "subsystems/subsystem_Intake.h"
#include "subsystems/subsystem_PoseTracker.h"

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class command_CubeShooter
    : public frc2::CommandHelper<frc2::CommandBase, command_CubeShooter> {
 public:
  command_CubeShooter( subsystem_Intake *intake, std::function<double()> TriggerInput);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;
 private:
   subsystem_Arm *m_Arm;
   subsystem_PoseTracker *m_PoseTracker;
   subsystem_Intake *m_Intake;
   std::function<double()> m_TriggerInput;
   double m_TopCubeAngle;
   double m_TopCubeVel;
   double m_MidCubeAngle;
   double m_MidCubeVel;
};
