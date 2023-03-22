// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/button/CommandXboxController.h>
#include "Constants.h"
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include "frc/XboxController.h"
#include "frc2/command/button/JoystickButton.h"
#include <frc/Joystick.h>

#include "commands/command_EveryBotIntake.h"
#include "commands/command_EveryBotOuttake.h"
#include "subsystems/subsystem_EveryBotIntake.h"
/**
 * This class is where the bulk of the robot should be declared.  Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls).  Instead, the structure of the robot (including subsystems,
 * commands, and trigger mappings) should be declared here.
 */
class RobotContainer {
 public:
  RobotContainer();

  frc2::Command* GetAutonomousCommand();
  

 private:
  // Replace with CommandPS4Controller or CommandJoystick if needed

  // The robot's subsystems are defined here...
  
  subsystem_EveryBotIntake m_EveryBotIntake;



  frc::XboxController XboxDrive{ControllerConstants::XboxDriveID};
  frc::XboxController XboxYaperator{ControllerConstants::XboxYaperatorID};


  frc::SendableChooser<frc2::Command*> m_Chooser;

  void ConfigureBindings();

};
