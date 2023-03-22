// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"
#include <frc2/command/button/Trigger.h>


RobotContainer::RobotContainer() {

  
  // Initialize all of your commands and subsystems here
  //std::function<double()> XDesired, std::function<double()> YDesired, std::function<double()> ThetaDesired
  // m_PoseTracker.SetDefaultCommand(command_AlignToDesired(&m_Drive, &m_PoseTracker, [this]{return 1;}, [this]{return 1;}, [this]{return 0;}));
  

 

  // Configure the button bindings
  ConfigureBindings();

}

void RobotContainer::ConfigureBindings() {


  frc2::JoystickButton(&XboxYaperator,
                       frc::XboxController::Button::kRightBumper)
                       .WhileTrue(command_EveryBotIntake(&m_EveryBotIntake).ToPtr());
  frc2::JoystickButton(&XboxYaperator,
                       frc::XboxController::Button::kLeftBumper)
                       .WhileTrue(command_EveryBotOuttake(&m_EveryBotIntake).ToPtr());


}

frc2::Command* RobotContainer::GetAutonomousCommand() {
  // An example command will be run in autonomous
  return m_Chooser.GetSelected();
}
