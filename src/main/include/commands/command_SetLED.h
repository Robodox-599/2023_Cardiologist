// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include "subsystems/subsystem_DriveTrain.h"
#include "subsystems/subsystem_Intake.h"
#include "subsystems/subsystem_LED.h"
#include "subsystems/subsystem_Arm.h"

#include "frc/XboxController.h"
#include <frc/Timer.h>


/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class command_SetLED
    : public frc2::CommandHelper<frc2::CommandBase, command_SetLED> {
 public:
  command_SetLED(subsystem_LED* LED,  frc::XboxController* DriverController, std::function<double()> WantsCube);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

  private:
  subsystem_LED* m_LED;
  // subsystem_Intake* m_Intake;
  frc::XboxController* m_DriverController;
  std::function<double()> m_WantsCube;

  frc::Timer m_Timer{};


};
