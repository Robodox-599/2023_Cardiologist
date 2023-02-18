// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"
#include <frc2/command/button/Trigger.h>
#include "commands/Autos.h"
#include "commands/ExampleCommand.h"

RobotContainer::RobotContainer() {
  m_Chooser.SetDefaultOption( "Taxi", m_TaxiAuto.get() );
  m_Chooser.AddOption( "Two Ball Auto", m_TwoScoreAuto.get() );
  // m_Chooser.SetDefaultOption("Test", "result of test");
  // m_Chooser.AddOption("Test2", "result of test2");
  frc::SmartDashboard::PutData(&m_Chooser);

  
  // Initialize all of your commands and subsystems here
  //std::function<double()> XDesired, std::function<double()> YDesired, std::function<double()> ThetaDesired
  // m_PoseTracker.SetDefaultCommand(command_AlignToDesired(&m_Drive, &m_PoseTracker, [this]{return 1;}, [this]{return 1;}, [this]{return 0;}));
  m_Drive.SetDefaultCommand( command_DriveTeleop(&m_Drive, &m_PoseTracker,
                                                       [this]{return -XboxDrive.GetRawAxis(ControllerConstants::xboxLYAxis);},
                                                       [this]{return -XboxDrive.GetRawAxis(ControllerConstants::xboxLXAxis);},
                                                       [this]{return -XboxDrive.GetRawAxis(ControllerConstants::xboxRXAxis);},
                                                       [this]{return XboxDrive.GetPOV();},
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

  frc2::JoystickButton(&XboxDrive, 
                        frc::XboxController::Button::kA)
      .WhileTrue(command_AlignToDesired(&m_Drive, &m_PoseTracker,
                                      [this]{return 2.0;},
                                      [this]{return 0.0;},
                                      [this]{return 180.0;}).ToPtr());


}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  // An example command will be run in autonomous
  return autos::Niemann(&m_Drive, &m_PoseTracker);
  // return autos::TestAuto(&m_Drive, "TestPath.wpilib.json", true);
}