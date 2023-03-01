// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

/**
 * The Constants header provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants.  This should not be used for any other
 * purpose.
 *
 * It is generally a good idea to place constants into subsystem- or
 * command-specific namespaces within this header, which can then be used where
 * they are needed.
 */
namespace ArmConstants {
    const int bottomArmMotorID = 32;
    const int bottomFollowerID = 33;
    const int topArmMotorID = 30;
    const int topFollowerID = 31;
    const int intakeTiltMotorID = 34;
    const int TopBrake1 = 0;
    const int TopBrake2 = 0;
    const int BottomBrake1 = 0;
    const int BottomBrake2 = 0;

    const double kBottomGoingUpP = 8.0; // change this
    const double kBottomGoingUpD = 000.0; // change this
    //  const double kBottomGoingUpF = 0.0000; // change this

    const double kBottomGoingDownP = 8.0; // change this
    const double kBottomGoingDownD = 000.0; // change this
    // const double kBottomGoingDownF = 0.0000
    ; // change this

    const double kTopGoingUpP = 0.25; // change this
    const double kTopGoingUpD = 0.0;  // change this
    // const double kTopGoingUpF = 0.1; // change this

    // const double kTopGoingDownP = 0.01; // change this
    // const double kTopGoingDownD = 0.0; // change this
    // const double kTopGoingDownF = 0.1; // change this

    //SmartMotion constants
    // const double kTopMaxA = 25.0; // change this
    // const double kBottomMaxA = 25.0;// change this 
    // const double kTopMaxV = 2500.0; // change this
    // const double kTopMinV = 0.0;
    // const double kBottomMaxV = 2500.0; // change this
    // const double kBottomMinV = 0.0; 
    const int ArmCurrentLimit = 12;

    const double intakeAngleConversion = 0.0;  // change this
    const double DegreesToRotations = 0.4423; // change this
    const double radiansToEncoder = 0.0; // change this
    const double AbsToRel = 16384/507 /*32.3156*/; 
    const double JoystickToArm = 1.0; // change this

    const double totalArmLength = 71.5;
    const double bottomJointLength = 38.0;
    const double topJointLength = 33.5;

    const double xOriginAdjustment = 0.0; // change this
    const double yOriginAdjustment = 0.0; // change this

    const double bufferZone = 0.0; // change this
    constexpr units::time::second_t ManualTimer{1.0};

    constexpr double TicksOffset = 0.0;  // change this
    constexpr double TicksToDegrees = (0.0 / 90);  // change this

    constexpr double HighConeX = 0.0;
    constexpr double HighConeY = 0.0;

    constexpr double MidConeX = 0.0;
    constexpr double MidConeY = 0.0;

    constexpr double HighCubeX = 0.0;
    constexpr double HighCubeY = 0.0;

    constexpr double MidCubeX = 0.0;
    constexpr double MidCubeY = 0.0;

    constexpr double RestX = 0.0;
    constexpr double RestY = 0.0;

    constexpr double SubstationX = 0.0;
    constexpr double SubstationY = 0.0;

    constexpr double GroundX = 0.0;
    constexpr double GroundY = 0.0;

    constexpr double ArmBackLimit = 0.0;
    constexpr double ArmFrontLimit = 0.0;
}

namespace ControllerConstants{
    constexpr double Deadband = 0.1;
    
    constexpr int XboxDriveID = 0;
    constexpr int XboxYaperatorID = 1;

    constexpr int xboxLXAxis = 0;
    constexpr int xboxLYAxis = 1;
    constexpr int xboxRXAxis = 4;
    constexpr int xboxRYAxis = 5;

    constexpr int xboxLTAxis = 2;
    constexpr int xboxRTAxis = 3;

    constexpr int xboxA = 1;
    constexpr int xboxB = 2;
    constexpr int xboxX = 3;
    constexpr int xboxY = 4;
    constexpr int xboxLB = 5;
    constexpr int xboxRB = 6;
    constexpr int xboxView = 7;
    constexpr int xboxMenu = 8;
    constexpr int xboxLeftJoyPress = 9;
    constexpr int xboxRightJoyPress = 10;
    constexpr int xboxRightDPad = 11;
}

namespace IntakeConstants {

constexpr int kDriverControllerPort = 0;
constexpr int IntakePistonLA = 0;
constexpr int IntakePistonLB = 0;
constexpr int IntakePistonRA = 0;
constexpr int IntakePistonRB = 0;

}  // namespace OperatorConstants
