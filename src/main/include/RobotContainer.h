// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/button/CommandXboxController.h>

#include "Constants.h"
#include "subsystems/ExampleSubsystem.h"
#include "subsystems/subsystem_DriveTrain.h"
#include "commands/command_DriveTeleop.h"
#include "commands/command_ShiftThrottle.h"
#include "commands/command_ZeroGyro.h"
#include "commands/command_StartStationBalance.h"
#include "commands/command_DriveAuton.h"

#include "frc/XboxController.h"
#include "frc2/command/button/JoystickButton.h"
#include <frc/Joystick.h>
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

  frc2::CommandPtr GetAutonomousCommand();
  

 private:
  // Replace with CommandPS4Controller or CommandJoystick if needed
  frc2::CommandXboxController m_driverController{
      0};

  // The robot's subsystems are defined here...
  ExampleSubsystem m_subsystem;
  subsystem_DriveTrain m_Drive;

  frc::XboxController XboxDrive{ControllerConstants::XboxDriveID};
  frc::Joystick XboxDriveBtns{ControllerConstants::XboxDriveID};
  frc::Joystick XboxYaperator{ControllerConstants::XboxYaperatorID};

  void ConfigureBindings();
};
