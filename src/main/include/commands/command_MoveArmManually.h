// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include "subsystems/subsystem_Arm.h"
#include <frc/Timer.h>
#include <frc/controller/ProfiledPIDController.h>

class command_MoveArmManually
    : public frc2::CommandHelper<frc2::CommandBase, command_MoveArmManually> {
 public:
  command_MoveArmManually(subsystem_Arm *arm, std::function<double()> leftJoystick, std::function<double()> rightJoystick, std::function<double()> triggers);
  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

 private:
  subsystem_Arm *m_arm;
  frc::Timer m_Timer;
  bool isDone = false;
  std::function<double()> m_LeftJoystickInput;
  std::function<double()> m_RightJoystickInput;
  std::function<double()> m_TriggerInput;
};
