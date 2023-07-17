// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandPtr.h>
#include "Constants.h"
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include "commands/command_ControllerVibrate.h"
#include "frc/XboxController.h"
#include "frc2/command/button/JoystickButton.h"
#include <frc/Joystick.h>
#include <frc2/command/button/CommandXboxController.h>


#include "subsystems/subsystem_DriveTrain.h"
#include "commands/command_DriveTeleop.h"
#include "commands/Autos.h"



#include "subsystems/subsystem_Arm.h"
#include "commands/command_MoveArmManually.h"
#include "commands/cGroup_Arm.h"

#include "subsystems/subsystem_LED.h"
#include "commands/command_SetLED.h"
#include "commands/command_Blink.h"

#include "subsystems/subsystem_Intake.h"
#include "commands/command_AutoClamp.h"

#include "subsystems/subsystem_GroundTake.h"
#include "commands/command_EngageGroundTake.h"

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
  subsystem_DriveTrain m_Drive;
  subsystem_PoseTracker m_PoseTracker;
  subsystem_Arm m_Arm;
  subsystem_Intake m_Intake;
  subsystem_LED m_LED;
  // subsystem_GroundTake m_GroundTake;
  // subsystem_EveryBotIntake m_EveryBotIntake;

  frc::XboxController XboxDrive{ControllerConstants::XboxDriveID};
  frc::XboxController XboxYaperator{ControllerConstants::XboxYaperatorID};

  frc::SendableChooser<frc2::Command*> m_Chooser;
  frc::Timer m_Timer{};

  frc2::CommandPtr m_TaxiAuto = autos::Taxi(&m_Drive, &m_PoseTracker);
  frc2::CommandPtr m_One_ScoreCubeAndTaxi = autos::One_ScoreCubeAndTaxi(&m_Drive, &m_PoseTracker, &m_Intake, &m_Arm);
  frc2::CommandPtr m_Two__ScoreCubeAndTaxi = autos::Two_ScoreCubeAndTaxi(&m_Drive, &m_PoseTracker, &m_Intake, &m_Arm);
  frc2::CommandPtr m_Three_ScoreCubeAndTaxi = autos::Three_ScoreCubeAndTaxi(&m_Drive, &m_PoseTracker, &m_Intake, &m_Arm);

  frc2::CommandPtr m_ScoreConeAndTaxi = autos::ScoreConeAndTaxi(&m_Drive, &m_PoseTracker, &m_Intake, &m_Arm);
  frc2::CommandPtr m_TwoTaxiAndBalance = autos::Two_TaxiAndBalance(&m_Drive, &m_PoseTracker);
  frc2::CommandPtr m_ThreeTaxiAndBalance = autos::Three_TaxiAndBalance(&m_Drive, &m_PoseTracker);
  frc2::CommandPtr m_Two_ScoreTaxiAndBalance = autos::Two_ScoreTaxiAndBalance(&m_Drive, &m_PoseTracker, &m_Intake, &m_Arm);

  frc2::CommandPtr m_ScoreCubeHigh = ArmMovements::HighCubeScoreAndStow(&m_Arm, &m_Intake);
  frc2::CommandPtr m_ScoreCubeMid = ArmMovements::MidCubeScoreAndStow(&m_Arm, &m_Intake);

  frc2::CommandPtr m_ScoreConeHigh = ArmMovements::HighConeScoreAndStow(&m_Arm, &m_Intake);
  frc2::CommandPtr m_ScoreConeMid = ArmMovements::MidConeScoreAndStow(&m_Arm, &m_Intake);

  // frc2::CommandPtr m_ThreeScoreAuto1 = autos::One_ThreeScoreAuto(&m_Drive, &m_PoseTracker, &m_Arm, &m_Intake, &m_GroundTake, &m_LED );
  // frc2::CommandPtr m_TwoScore = autos::One_TwoScore(&m_Drive, &m_PoseTracker, &m_Arm, &m_Intake, &m_GroundTake, &m_LED  );
  // frc2::CommandPtr m_TwoScoreAndBalance = autos::One_TwoScoreAndBalance(&m_Drive, &m_PoseTracker, &m_Arm, &m_Intake, &m_GroundTake, &m_LED  );
  // frc2::CommandPtr m_ThreeScoreAndBalance = autos::One_ThreeScoreAndBalance(&m_Drive, &m_PoseTracker, &m_Arm, &m_Intake, &m_GroundTake, &m_LED  );

  // frc2::CommandPtr m_One_ScoreIntakeScore = autos::One_ScoreIntakeScore(&m_Drive, &m_PoseTracker, &m_Intake, &m_Arm);
  // frc2::CommandPtr m_TestPickUp = autos::TestPickUp(&m_Drive, &m_PoseTracker, &m_Intake, &m_Arm);
  void ConfigureBindings();
};
