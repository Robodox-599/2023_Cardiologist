// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include <frc/Timer.h>
#include "frc/XboxController.h"
#include "subsystems/subsystem_Intake.h"


/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class command_ControllerVibrate
    : public frc2::CommandHelper<frc2::CommandBase, command_ControllerVibrate> {
 public:
  command_ControllerVibrate(frc::XboxController *Drive, subsystem_Intake *Intake, frc::Timer *Timer);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;
 private:
  frc::XboxController *m_Driver;
  subsystem_Intake *m_Intake;
  frc::Timer *m_Timer;
};
