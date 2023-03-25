// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"
#include <frc2/command/button/Trigger.h>
#include "commands/Autos.h"
#include "commands/ExampleCommand.h"

RobotContainer::RobotContainer() {
  m_Chooser.SetDefaultOption( "Taxi", m_TaxiAuto.get());
  m_Chooser.AddOption( "One_ScoreAndTaxi", m_OneScoreAndTaxi.get() );
  m_Chooser.AddOption("Two_ScoreAndTaxi", m_TwoScoreAndTaxi.get());
  m_Chooser.AddOption("Three_ScoreAndTaxi", m_ThreeScoreAndTaxi.get());
  m_Chooser.AddOption("Two_TaxiAndBalance", m_TwoTaxiAndBalance.get());
  m_Chooser.AddOption("Three_TaxiAndBalance", m_ThreeTaxiAndBalance.get());
  m_Chooser.AddOption("Two_ScoreTaxiAndBalance", m_Two_ScoreTaxiAndBalance.get());
  m_Chooser.AddOption("NonAuto", nullptr);
  m_Chooser.AddOption("ScoreHighCube", m_ScoreHigh.get());
  // // m_Chooser.SetDefaultOption("Test", "result of test");
  frc::SmartDashboard::PutData(&m_Chooser);

  
  // Initialize all of your commands and subsystems here
  //std::function<double()> XDesired, std::function<double()> YDesired, std::function<double()> ThetaDesired
  // m_PoseTracker.SetDefaultCommand(command_AlignToDesired(&m_Drive, [this]{return 1;}, [this]{return 1;}, [this]{return 0;}));
  m_Drive.SetDefaultCommand( command_DriveTeleop(&m_Drive, &m_PoseTracker,
                                                       [this]{return -XboxDrive.GetRawAxis(ControllerConstants::xboxLYAxis);},
                                                       [this]{return -XboxDrive.GetRawAxis(ControllerConstants::xboxLXAxis);},
                                                       [this]{return -XboxDrive.GetRawAxis(ControllerConstants::xboxRXAxis);},
                                                       [this]{return XboxDrive.GetLeftBumperPressed();},
                                                       [this]{return XboxDrive.GetRightBumperPressed();},
                                                       [this]{return SwerveConstants::IsFieldRelative;},
                                                       [this]{return SwerveConstants::IsOpenLoop;}));

  m_LED.SetDefaultCommand( command_SetLED(&m_LED,
                                                       [this]{return XboxDrive.GetRawAxis(ControllerConstants::xboxLTAxis) 
                                                                        - XboxDrive.GetRawAxis(ControllerConstants::xboxRTAxis);}));

  m_Arm.SetDefaultCommand(command_MoveArmManually(&m_Arm,
                        [this]{return XboxYaperator.GetRawAxis(ControllerConstants::xboxLYAxis);},
                        [this]{return XboxYaperator.GetRawAxis(ControllerConstants::xboxRYAxis);},
                        [this]{return XboxYaperator.GetRightTriggerAxis() - XboxYaperator.GetLeftTriggerAxis();}));

  // m_Intake.SetDefaultCommand(command_ControllerVibrate(&XboxDrive, &m_Intake, &m_Timer));

  // Configure the button bindings
  ConfigureBindings();

}

void RobotContainer::ConfigureBindings() {
  frc2::JoystickButton(&XboxDrive,
                       frc::XboxController::Button::kRightStick)
      .OnTrue(command_ShiftThrottle(&m_Drive).ToPtr());


  frc2::JoystickButton(&XboxDrive,
                        frc::XboxController::Button::kY)
      .OnTrue(command_ZeroGyro(&m_Drive).ToPtr());


  // frc2::JoystickButton(&XboxDrive,
  //                       frc::XboxController::Button::kA)
  //     .WhileTrue(command_AlignToDesired(&m_Drive,
  //                                     [this]{return 0.0;},
  //                                     [this]{return 0.0;},
  //                                     [this]{return 0.0;}).ToPtr());
  //  frc2::JoystickButton(&XboxDrive, 
  //                        frc::XboxController::Button::kB)
  //      .OnTrue(command_Balance(&m_Drive).ToPtr());


  // frc2::JoystickButton(&XboxDrive,
  //                     frc::XboxController::Button::kA)
  //     .OnTrue(command_ToggleTiltCorrection(&m_Drive).ToPtr());

  // frc2::JoystickButton(&XboxDrive,
  //                     frc::XboxController::Button::kB)
  //     .OnTrue(command_SetLED(&m_Drive, [=]{return false;}).ToPtr());

  // frc2::JoystickButton(&XboxDrive, 
  //                       frc::XboxController::Button::kA)
  //     .WhileTrue(command_AlignToDesired(&m_Drive,
  //                                     [this]{return 2.0;},
  //                                     [this]{return 0.0;},
                                      // [this]{return 180.0;}).ToPtr());
  frc2::JoystickButton(&XboxYaperator, 
                       frc::XboxController::Button::kA)
                       .OnTrue(ArmMovements::ToMidCube(&m_Arm));

  frc2::JoystickButton(&XboxYaperator, 
                       frc::XboxController::Button::kX)
                       .OnTrue(ArmMovements::ToHighCube(&m_Arm));



  frc2::JoystickButton(&XboxYaperator, 
                       frc::XboxController::Button::kY)
                       .OnTrue(ArmMovements::ToHighCone(&m_Arm));

  frc2::JoystickButton(&XboxYaperator, 
                       frc::XboxController::Button::kB)
                       .OnTrue(ArmMovements::ToMidCone(&m_Arm));

  frc2::JoystickButton(&XboxYaperator, 
                       frc::XboxController::Button::kStart)
                       .OnTrue(ArmMovements::ToSubstation(&m_Arm));

  frc2::JoystickButton(&XboxYaperator, 
                       frc::XboxController::Button::kBack)
                       .OnTrue(ArmMovements::ToStow(&m_Arm));

  frc2::JoystickButton(&XboxYaperator, 
                       frc::XboxController::Button::kRightStick)
                       .OnTrue(ArmMovements::ToPortal(&m_Arm));  


  frc2::JoystickButton(&XboxYaperator,
                       frc::XboxController::Button::kRightBumper)
                       .OnTrue(command_IntakeObject(&m_Intake).ToPtr());
  frc2::JoystickButton(&XboxYaperator,
                       frc::XboxController::Button::kLeftBumper)
                       .OnTrue(command_OuttakeObject(&m_Intake).ToPtr());
  frc2::JoystickButton(&XboxYaperator,
                       frc::XboxController::Button::kLeftStick)
                       .OnTrue(command_ToggleClamp(&m_Intake).ToPtr());
  
}

frc2::Command* RobotContainer::GetAutonomousCommand() {
  // An example command will be run in autonomous
  return m_Chooser.GetSelected();
    // return null;
}
