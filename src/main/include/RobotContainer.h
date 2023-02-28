// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/button/CommandXboxController.h>
#include "subsystems/subsystem_PoseTracker.h"
#include "Constants.h"
#include "commands/command_VisionPose.h"
#include "subsystems/ExampleSubsystem.h"
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/smartdashboard/SmartDashboard.h>


#include "subsystems/subsystem_DriveTrain.h"
#include "commands/command_DriveTeleop.h"
#include "commands/command_ShiftThrottle.h"
#include "commands/command_ZeroGyro.h"
#include "commands/command_AlignToDesired.h"
#include "commands/Autos.h"
#include "subsystems/subsystem_Arm.h"
#include "commands/command_MoveArm.h"
#include "commands/command_MoveArmManually.h"
#include "frc/XboxController.h"
#include "frc2/command/button/JoystickButton.h"
#include <frc/Joystick.h>

#include "subsystems/subsystem_Intake.h"

#include "commands/command_OuttakeObject.h"
#include "commands/command_ToggleClamp.h"
#include "commands/command_IntakeObject.h"
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

  // The robot's subsystems are defined here...
  ExampleSubsystem m_subsystem;
  subsystem_DriveTrain m_Drive;
  subsystem_PoseTracker m_PoseTracker;
  subsystem_Arm m_Arm;
  subsystem_Intake m_Intake;


  frc::XboxController XboxDrive{ControllerConstants::XboxDriveID};
  frc::XboxController xboxYaperator{ControllerConstants::XboxYaperatorID};


  frc::SendableChooser<frc2::Command*> m_Chooser;

  frc2::CommandPtr m_TaxiAuto = autos::Kasparov(&m_Drive, &m_PoseTracker);
  frc2::CommandPtr m_TwoScoreAuto = autos::Niemann(&m_Drive, &m_PoseTracker);

  void ConfigureBindings();


};
