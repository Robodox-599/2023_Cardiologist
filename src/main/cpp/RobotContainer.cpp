// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"
#include <frc2/command/button/Trigger.h>
#include "commands/Autos.h"
#include "commands/ExampleCommand.h"

RobotContainer::RobotContainer() {
  // Initialize all of your commands and subsystems here
  m_Drive.SetDefaultCommand( command_DriveTeleop(&m_Drive,
                                                       [this]{return -XboxDrive.GetRawAxis(ControllerConstants::xboxLYAxis);},
                                                       [this]{return -XboxDrive.GetRawAxis(ControllerConstants::xboxLXAxis);},
                                                       [this]{return -XboxDrive.GetRawAxis(ControllerConstants::xboxRXAxis);},
                                                       [this]{return SwerveConstants::IsFieldRelative;},
                                                       [this]{return SwerveConstants::IsOpenLoop;}));
  // Configure the button bindings
  ConfigureBindings();
}

void RobotContainer::ConfigureBindings() {
  frc2::JoystickButton(&XboxDrive,
                       frc::XboxController::Button::kX)
      .OnTrue(command_ShiftThrottle(&m_Drive).ToPtr());

  frc2::JoystickButton(&XboxDrive,
                        frc::XboxController::Button::kY)
      .OnTrue(command_ZeroGyro(&m_Drive).ToPtr());



}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  // An example command will be run in autonomous
  return autos::TestAuto(&m_Drive, frc::DriverStation::Alliance::kBlue);
  // return autos::TestAuto(&m_Drive, "TestPath.wpilib.json", true);
}
 