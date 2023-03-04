// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc2/command/button/Trigger.h>

#include "commands/Autos.h"
#include "commands/ExampleCommand.h"

RobotContainer::RobotContainer(){

  
  m_Arm.SetDefaultCommand(command_MoveArmManually(&m_Arm,
                        [this]{return controller.GetRawAxis(ControllerConstants::xboxLYAxis);},
                        [this]{return controller.GetRawAxis(ControllerConstants::xboxRYAxis);},
                        [this]{return controller.GetRightTriggerAxis() - controller.GetLeftTriggerAxis();}));
  ConfigureBindings();
}

void RobotContainer::ConfigureBindings() {

  frc2::JoystickButton(&controller, 
                       frc::XboxController::Button::kA)
                       .OnTrue(ArmMovements::ToMidCube(&m_Arm));

  frc2::JoystickButton(&controller, 
                       frc::XboxController::Button::kX)
                       .OnTrue(ArmMovements::ToHighCube(&m_Arm));
                  
  frc2::JoystickButton(&controller, 
                       frc::XboxController::Button::kY)
                       .OnTrue(ArmMovements::ToHighCone(&m_Arm));

  frc2::JoystickButton(&controller, 
                       frc::XboxController::Button::kB)
                       .OnTrue(ArmMovements::ToMidCone(&m_Arm));

  frc2::JoystickButton(&controller, 
                       frc::XboxController::Button::kStart)
                       .OnTrue(ArmMovements::ToSubstation(&m_Arm));

  frc2::JoystickButton(&controller, 
                       frc::XboxController::Button::kBack)
                       .OnTrue(ArmMovements::ToStow(&m_Arm));

  frc2::JoystickButton(&controller, 
                       frc::XboxController::Button::kRightStick)
                       .OnTrue(ArmMovements::ToGround(&m_Arm));

  // frc2::JoystickButton(&controller, 
  //                       frc::XboxController::Button::kA)
  //                       .OnTrue(command_MoveArm(&m_Arm, 
  //                       [=]{return ArmConstants::MidCubeShoulder;}, 
  //                       [=]{return ArmConstants::MidCubeElbow;},
  //                       [=]{return ArmConstants::MidCubeTilt;}).ToPtr());

//   frc2::JoystickButton(&controller, 
//                         frc::XboxController::Button::kX)
//                         .OnTrue(command_MoveArm(&m_Arm, 
//                         [=]{return ArmConstants::HighCubeShoulder;}, 
//                         [=]{return ArmConstants::HighCubeElbow;},
//                         [=]{return ArmConstants::HighCubeTilt;}).ToPtr());

//   frc2::JoystickButton(&controller, 
//                         frc::XboxController::Button::kY)
//                         .OnTrue(command_MoveArm(&m_Arm, 
//                         [=]{return ArmConstants::HighConeShoulder;}, 
//                         [=]{return ArmConstants::HighConeElbow;},
//                         [=]{return ArmConstants::HighConeTilt;}).ToPtr());

//   frc2::JoystickButton(&controller, 
//                         frc::XboxController::Button::kB)
//                         .OnTrue(command_MoveArm(&m_Arm, 
//                         [=]{return ArmConstants::MidConeShoulder;}, 
//                         [=]{return ArmConstants::MidConeElbow;},
//                         [=]{return ArmConstants::MidConeTilt;}).ToPtr());

//   frc2::JoystickButton(&controller, 
//                         frc::XboxController::Button::kStart)
//                         .OnTrue(command_MoveArm(&m_Arm, 
//                         [=]{return ArmConstants::SubstationShoulder;}, 
//                         [=]{return ArmConstants::SubstationElbow;},
//                         [=]{return ArmConstants::SubstationTilt;}).ToPtr());

//   frc2::JoystickButton(&controller, 
//                         frc::XboxController::Button::kBack)
//                         .OnTrue(command_MoveArm(&m_Arm, 
//                         [=]{return ArmConstants::StowShoulder;}, 
//                         [=]{return ArmConstants::StowElbow;},
//                         [=]{return ArmConstants::StowTilt;}).ToPtr());

//   frc2::JoystickButton(&controller, 
//                         frc::XboxController::Button::kBack)
//                         .OnTrue(command_MoveArm(&m_Arm, 
//                         [=]{return ArmConstants::StowShoulder;}, 
//                         [=]{return ArmConstants::StowElbow;},
//                         [=]{return ArmConstants::StowTilt;}).ToPtr());

//   // frc2::JoystickButton(&controller, 
//   //                       frc::XboxController::Button::kRightStick)
//   //                       .OnTrue(command_MoveArm(&m_Arm, 
//   //                       [=]{return ArmConstants::GroundShoulder;}, 
//   //                       [=]{return ArmConstants::GroundElbow;},
//   //                       [=]{return ArmConstants::GroundTilt;}).ToPtr());
                      

//   frc2::JoystickButton(&controller, 
//                         frc::XboxController::Button::kRightStick)
//                         .OnTrue(command_ArmInputSwitch(&m_Arm).ToPtr());
            
 }