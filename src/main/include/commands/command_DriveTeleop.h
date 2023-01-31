// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/subsystem_DriveTrain.h"
#include "Constants.h"
#include "units/length.h"
/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class command_DriveTeleop
    : public frc2::CommandHelper<frc2::CommandBase, command_DriveTeleop> {
 public:
  command_DriveTeleop(subsystem_DriveTrain* DriveTrain,
                            std::function<double()> xSpeed,
                            std::function<double()> ySpeed,
                            std::function<double()> zRotation,
                            std::function<bool()> FieldRelative,
                            std::function<bool()> OpenLoop);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

  private:
  subsystem_DriveTrain* m_DriveTrain;
  std::function<double()> m_xSpeed;
  std::function<double()> m_ySpeed;
  std::function<double()> m_zRotation;
  std::function<bool()> m_FieldRelative;
  std::function<bool()> m_OpenLoop;

};
