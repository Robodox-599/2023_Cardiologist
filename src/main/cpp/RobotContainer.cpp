// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc2/command/button/Trigger.h>
#include "commands/Autos.h"
#include "commands/ExampleCommand.h"

RobotContainer::RobotContainer() : m_IntakeRun(&m_Intake, [=]{return 0.0;}) , m_IntakeClamp(&m_Intake)
{
  // Initialize all of your commands and subsystems here
  m_Intake.SetDefaultCommand(command_IntakeRun(&m_Intake, [this] {return xbox.GetRawAxis(ControllerConstants::XboxLTAxis) - xbox.GetRawAxis(ControllerConstants::XboxRTAxis);}));
  // Configure the button bindings
  ConfigureBindings();
}

void RobotContainer::ConfigureBindings() {
  // Configure your trigger bindings here
  frc2::JoystickButton(&xbox, 
                       frc::XboxController::Button::kA)
                       .OnTrue(command_IntakeClamp(&m_Intake).ToPtr());
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  // An example command will be run in autonomous
  return autos::ExampleAuto(&m_subsystem);
}
