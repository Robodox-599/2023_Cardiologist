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

    const int PDH_ID  = 0;

    const int ShoulderMotorID = 32;
    const int ShoulderFollowerID = 33;
    const int ElbowMotorID = 30;
    const int ElbowFollowerID = 31;
    const int WristMotorID = 34;
    const int ElbowBrake1 = 0;
    const int ElbowBrake2 = 0;
    const int ShoulderBrake1 = 0;
    const int ShoulderBrake2 = 0;

    const double kShoulderP = 8.0; 
    const double kShoulderI = 0.0;
    const double kShoulderD = 0.0; 
    const int kShoulderSlot = 0;
    const double kShoulderIZone = 1.0;

    const double kElbowUpP = 0.04; 
    const double kElbowUpI = 0.0005;
    const double kElbowUpD = 0.0; 
    const int kElbowUpSlot = 0;

    const double kElbowDownP = 0.05;
    const double kElbowDownI = 0.0;
    const double kElbowDownD = 0.0; 
    const int kElbowDownSlot = 1;
    const double kElbowIZone = 1.0;

    const double kWristP = 1.0;
    const double kWristI = 0.0;
    const double kWristD = 0.0;

    // SmartMotion constants
    const double kElbowSmartMotionP = 0.0001;
    const double kElbowSmartMotionD = 0.0;
    const double kElbowSmartMotionFF = 0.03;
    const double kElbowMaxA = 80.0; 
    const double kElbowMaxV = 80.0; 
    const double kElbowMinV = 0.0;
    const double kShoulderMaxA = 25.0; 
    const double kShoulderMaxV = 2500.0; 
    const double kShoulderMinV = 0.0; 
    const int ArmCurrentLimit = 12;
    const int DownwardElbowCurrentLimit = 3;

    const double intakeAngleConversion = 0.0;  
    const double DegreesToRotations = 0.4423; 
    const double radiansToEncoder = 0.0; 
    const double AbsToRel = 16384/507; 
    const double JoystickToArm = 1.0; 

    //units are in meters
    const double totalArmLength = 71.5;
    const double ShoulderJointLength = 0.965;
    const double ElbowJointLength = 0.838;
    const double IntakeJointLength = 0.371;

    //units are in kg
    const double ShoulderJointMass = 3.896;
    const double ElbowJointMass = 2.22;
    const double IntakeJointMass = 3.18;

    const double xOriginAdjustment = 0.0; 
    const double yOriginAdjustment = 0.0; 

    const double bufferZone = 0.1; 
    constexpr units::time::second_t ManualTimer{1.0};

    constexpr double TicksOffset = 0.0;  
    constexpr double TicksToDegrees = (0.0 / 90);  

    constexpr double HighConeX = 0.0;
    constexpr double HighConeY = 0.0;
    constexpr double HighConeShoulder = -16.0;
    constexpr double HighConeElbow = 33.2;
    constexpr double HighConeTilt = 2.0;

    constexpr double MidConeX = 0.0;
    constexpr double MidConeY = 0.0;
    constexpr double MidConeShoulder = -8.1;
    constexpr double MidConeElbow = 21.5;
    constexpr double MidConeTilt = 2.0;

    constexpr double HighCubeX = 0.0;
    constexpr double HighCubeY = 0.0;
    constexpr double HighCubeShoulder = -11.5;
    constexpr double HighCubeElbow = 24.5;
    constexpr double HighCubeTilt = 0.0;

    constexpr double MidCubeX = 0.0;
    constexpr double MidCubeY = 0.0;
    constexpr double MidCubeShoulder = -3.0;
    constexpr double MidCubeElbow = 14.0;
    constexpr double MidCubeTilt = 0.0;

    constexpr double RestX = 0.0;
    constexpr double RestY = 0.0;
    constexpr double StowShoulder = -1.0;
    constexpr double StowElbow = 1.0;
    constexpr double StowTilt = 0.0;

    constexpr double SubstationX = 0.0;
    constexpr double SubstationY = 0.0;
    constexpr double SubstationShoulder = 0.0;
    constexpr double SubstationElbow = 25.5;
    constexpr double SubstationTilt = 0.0;

    constexpr double GroundX = 0.0;
    constexpr double GroundY = 0.0;
    constexpr double GroundShoulder = -26.8;
    constexpr double GroundElbow = -14.5; 
    constexpr double GroundTilt = 0.0; 

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
