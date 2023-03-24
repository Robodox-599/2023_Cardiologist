// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/subsystem_Intake.h"
#include "subsystems/subsystem_LED.h"

#include <frc/Timer.h>
#include "frc/XboxController.h"
/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class command_ManipulateIntake
    : public frc2::CommandHelper<frc2::CommandBase, command_ManipulateIntake> {
 public:
  command_ManipulateIntake(subsystem_Intake* Intake, subsystem_LED* LED, frc::XboxController* DriveController, std::function<double()> WantsCube, std::function<bool()> IsIntake, std::function<bool()> IsOutake);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;
  private:
  subsystem_Intake* m_Intake;
  subsystem_LED* m_LED;
  frc::XboxController* m_DriveController;
  std::function<double()> m_WantsCube;
  std::function<bool()> m_IsIntake;
  std::function<bool()> m_IsOutake;
};
