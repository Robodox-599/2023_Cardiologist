// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc2/command/button/Trigger.h>

#include "commands/Autos.h"
#include "commands/ExampleCommand.h"

RobotContainer::RobotContainer(){
  ConfigureBindings();
}

//hellllllloooooooooooooooooooooooooo

void RobotContainer::ConfigureBindings() {
  frc2::JoystickButton(&controller, 
                        frc::XboxController::Button::kLeftStick)
                        .OnTrue(command_MoveArmManually(&m_Arm,
                        [this]{return controller.GetRawAxis(ControllerConstants::xboxLYAxis);},
                        [this]{return controller.GetRawAxis(ControllerConstants::xboxRYAxis);}).ToPtr());

  frc2::JoystickButton(&controller, 
                        frc::XboxController::Button::kRightStick)
                        .OnTrue(command_MoveArmManually(&m_Arm,
                        [this]{return controller.GetRawAxis(ControllerConstants::xboxLYAxis);},
                        [this]{return controller.GetRawAxis(ControllerConstants::xboxRYAxis);}).ToPtr());

  frc2::JoystickButton(&controller, 
                        frc::XboxController::Button::kA)
                        .OnTrue(command_MoveArm(&m_Arm, [=]{return ArmConstants::MidCubeX;}, [=]{return ArmConstants::MidCubeY;}).ToPtr());

  frc2::JoystickButton(&controller, 
                        frc::XboxController::Button::kX)
                        .OnTrue(command_MoveArm(&m_Arm, [=]{return ArmConstants::HighCubeX;}, [=]{return ArmConstants::HighCubeY;}).ToPtr());

  frc2::JoystickButton(&controller, 
                        frc::XboxController::Button::kY)
                        .OnTrue(command_MoveArm(&m_Arm, [=]{return ArmConstants::HighConeX;}, [=]{return ArmConstants::HighConeY;}).ToPtr());

  frc2::JoystickButton(&controller, 
                        frc::XboxController::Button::kB)
                        .OnTrue(command_MoveArm(&m_Arm, [=]{return ArmConstants::MidConeX;}, [=]{return ArmConstants::MidConeY;}).ToPtr());

  frc2::JoystickButton(&controller, 
                        frc::XboxController::Button::kStart)
                        .OnTrue(command_MoveArm(&m_Arm, [=]{return ArmConstants::SubstationX;}, [=]{return ArmConstants::SubstationY;}).ToPtr());

  frc2::JoystickButton(&controller, 
                        frc::XboxController::Button::kBack)
                        .OnTrue(command_MoveArm(&m_Arm, [=]{return ArmConstants::RestX;}, [=]{return ArmConstants::RestY;}).ToPtr());

  frc2::JoystickButton(&controller, 
                        frc::XboxController::Button::kRightStick)
                        .OnTrue(command_MoveArm(&m_Arm, [=]{return ArmConstants::GroundX;}, [=]{return ArmConstants::GroundY;}).ToPtr());
                      
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  // An example command will be run in autonomous
  return autos::ExampleAuto(&m_subsystem);
}
