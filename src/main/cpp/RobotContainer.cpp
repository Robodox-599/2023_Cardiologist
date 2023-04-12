// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"
#include <frc2/command/button/Trigger.h>
#include "commands/Autos.h"

RobotContainer::RobotContainer()
 {
  m_Chooser.SetDefaultOption( "Taxi", m_TaxiAuto.get());
  m_Chooser.AddOption( "One_ScoreAndTaxi", m_OneScoreAndTaxi.get() );
  m_Chooser.AddOption("Two_ScoreAndTaxi", m_TwoScoreAndTaxi.get());
  m_Chooser.AddOption("Three_ScoreAndTaxi", m_ThreeScoreAndTaxi.get());
  m_Chooser.AddOption("Two_TaxiAndBalance", m_TwoTaxiAndBalance.get());
  m_Chooser.AddOption("Three_TaxiAndBalance", m_ThreeTaxiAndBalance.get());
  m_Chooser.AddOption("Two_ScoreTaxiAndBalance", m_Two_ScoreTaxiAndBalance.get());
  m_Chooser.AddOption("NonAuto", nullptr);
  m_Chooser.AddOption("ScoreHighCube", m_ScoreCubeHigh.get());
  m_Chooser.AddOption("ScoreMidCube", m_ScoreCubeMid.get());
  m_Chooser.AddOption("ScoreHighCone", m_ScoreConeHigh.get());
  m_Chooser.AddOption("ScoreMidCone", m_ScoreConeMid.get());
  m_Chooser.AddOption("Three Score Auto 1", m_ThreeScoreAuto1.get());
  m_Chooser.AddOption("Three Score Auto 2", m_ThreeScoreAuto2.get());
  // m_Chooser.AddOption("TestPickup", m_TestPickUp.get());
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
                        [this]{return XboxYaperator.GetRightTriggerAxis() - XboxYaperator.GetLeftTriggerAxis();},
                        [this]{return XboxYaperator.GetPOV();}));

  // m_Intake.SetDefaultCommand(command_ControllerVibrate(&XboxDrive, &m_Intake, &m_Timer));

  // Configure the button bindings
  ConfigureBindings();

}

void RobotContainer::ConfigureBindings() {

  
  frc2::JoystickButton(&XboxDrive,
                       frc::XboxController::Button::kRightStick)
      .OnTrue(m_Drive.ToggleThrottleCommand());


  frc2::JoystickButton(&XboxDrive,
                        frc::XboxController::Button::kY)
      .OnTrue(m_Drive.ZeroGyroCommand());


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
                       frc::XboxController::Button::kY)
                       .OnTrue(ArmMovements::ToHighCone(&m_Arm));

  frc2::JoystickButton(&XboxYaperator, 
                       frc::XboxController::Button::kB)
                       .OnTrue(ArmMovements::ToMidCone(&m_Arm));

  frc2::JoystickButton(&XboxYaperator, 
                       frc::XboxController::Button::kX)
                       .OnTrue(ArmMovements::ToHighCube(&m_Arm));

  frc2::JoystickButton(&XboxYaperator, 
                       frc::XboxController::Button::kA)
                       .OnTrue(ArmMovements::ToMidCube(&m_Arm));

  frc2::JoystickButton(&XboxYaperator, 
                       frc::XboxController::Button::kBack)
                       .OnTrue(ArmMovements::TiltedStow(&m_Arm));

  frc2::JoystickButton(&XboxYaperator, 
                       frc::XboxController::Button::kStart)
                       .OnTrue(ArmMovements::ToPortal(&m_Arm));

  frc2::JoystickButton(&XboxYaperator,
                       frc::XboxController::Button::kRightBumper)
                       .OnTrue(command_AutoClamp(&m_Intake).ToPtr());

  frc2::JoystickButton(&XboxYaperator,
                       frc::XboxController::Button::kLeftBumper)
                       .OnTrue(m_Intake.UnclampCommand());

    frc2::JoystickButton(&XboxYaperator, 
                       frc::XboxController::Button::kRightStick)
                       .OnTrue(m_Intake.ClampCommand());  

  frc2::JoystickButton(&XboxYaperator,
                       frc::XboxController::Button::kLeftStick)
                       .OnTrue(ArmMovements::ToGround(&m_Arm, &m_Intake));

  
  
  //Score and Stow (not in use)
  // frc2::JoystickButton(&XboxYaperator, 
  //                      frc::XboxController::Button::kX)
  //                      .OnTrue(ArmMovements::HighCubeScoreAndStow(&m_Arm, &m_Intake));

  // frc2::JoystickButton(&XboxYaperator, 
  //                      frc::XboxController::Button::kA)
  //                      .OnTrue(ArmMovements::MidCubeScoreAndStow(&m_Arm, &m_Intake));
                       
  // frc2::JoystickButton(&XboxYaperator, 
  //                      frc::XboxController::Button::kY)
  //                      .OnTrue(ArmMovements::HighConeScoreAndStow(&m_Arm, &m_Intake));

  // frc2::JoystickButton(&XboxYaperator, 
  //                      frc::XboxController::Button::kB)
  //                      .OnTrue(ArmMovements::MidConeScoreAndStow(&m_Arm, &m_Intake));
}


frc2::Command* RobotContainer::GetAutonomousCommand() {
  // An example command will be run in autonomous
  return m_Chooser.GetSelected();
    // return null;
}
