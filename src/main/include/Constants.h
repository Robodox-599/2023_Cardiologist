// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once
#include <units/length.h>
#include <units/voltage.h>
#include <units/velocity.h>
#include <units/time.h>
#include <units/acceleration.h>
#include <units/angular_acceleration.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/trajectory/TrapezoidProfile.h>
#include <frc/controller/PIDController.h>
#include <frc/trajectory/TrapezoidProfile.h>



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

    const double kBottomGoingUpP = 8; // change this
    const double kBottomGoingUpD = 000.0; // change this
    //  const double kBottomGoingUpF = 0.0000; // change this

    const double kBottomGoingDownP = 8; // change this
    const double kBottomGoingDownD = 000.0; // change this
    // const double kBottomGoingDownF = 0.0000
    ; // change this

    const double kTopGoingUpP = 0.01; // change this
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
    const int ArmCurrentLimit = 15;

    const double intakeAngleConversion = 0.0;  // change this
    const double DegreesToRotations = 0.4423; // change this
    const double radiansToEncoder = 0.0; // change this
    const double AbsToRel = 16384/507 /*32.3156*/; 
    const double JoystickToArm = 0.0; // change this

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


namespace SwerveConstants{

    constexpr int BalancekP = 0;
    constexpr int BalancekI = 0;
    constexpr int BalancekD = 0;

    constexpr int CANCoderID = 12; 
    constexpr bool InvertGyro = false;

    /*Drivetrain constants*/
    constexpr double OpenLoopRamp = 0.25;
    constexpr double closedLoopRamp = 0.375;

    constexpr double DriveGearRatio = 6.75;
    constexpr double AngleGearRatio = 150.0 / 7.0;

    
    constexpr units::meter_t WheelCircumference{ 4.0_in * M_PI  };

    const frc::Translation2d m_FrontLeft{11.375_in, 11.375_in};
    const frc::Translation2d m_FrontRight{11.375_in, -11.375_in};
    const frc::Translation2d m_BackLeft{-11.375_in, 11.375_in};
    const frc::Translation2d m_BackRight{-11.375_in, -11.375_in};

    const frc::SwerveDriveKinematics<4> m_kinematics{m_FrontLeft,
                                               m_FrontRight,
                                               m_BackLeft,
                                               m_BackRight};




    //Change to non-linear throttle for finer tuned movements
    enum Throttle {
        LINEAR = 1,
        NONLINEAR = 2
    };



    //Tip Correction PID (PITCH)
    constexpr double PitchKP = 0.0;
    constexpr double PitchKD = 0.0;


    //Tip Correction PID (ROLL) 
    constexpr double RollKP = 0.0;
    constexpr double RollKD = 0.0;
    

    /*setting up correct units for the simepleMotorFeedforward KS gain*/
    constexpr units::volt_t DriveKS{0.1646};

    constexpr units::volt_t VoltageKV{2.239};
    constexpr units::foot_t FeetKV{1.0};
    constexpr units::second_t TimeKV{1.0};
    /*Setting up correct units for the simpleMotorFeedforward KV gain
    Change VoltageKV when wanting to change 
    the KV gain*/
    constexpr auto DriveKV = VoltageKV * TimeKV / FeetKV;

    constexpr units::volt_t VoltageKA{0.73308};
    constexpr units::foot_t FeetKA{1.0};
    constexpr units::second_t TimeKA{1.0};
    /*Setting up correct units for the simpleMotorFeedforward KA gain
    Change VoltageKA when wanting to change the KA gain*/
    constexpr auto DriveKA = VoltageKA * (TimeKA * TimeKA) / FeetKA; 
    constexpr units::volt_t kNominal {12.0};
    /*Angle Encoder Invert*/
    constexpr bool CanCoderInvert = false;

    /*Swerve Angle Motor PID gains*/
    constexpr double AngleKP = 0.15;
    constexpr double AngleKI = 0.0;
    constexpr double AngleKD = 0.00;
    constexpr double AngleKF = 0.0;

    /*Swerve Angle Current Limit Config*/
    constexpr bool AngleEnableCurrentLimit = true;
    constexpr int AngleContinuousCurrentLimit = 10;
    constexpr int AnglePeakCurrentLimit = 20;
    constexpr double AnglePeakCurrentDuration = 0.1;
    
    /*Swerve Drive Motor PID gains*/
    constexpr double DriveKP = 0.1;
    constexpr double DriveKI = 0.0;
    constexpr double DriveKD = 0.00;
    constexpr double DriveKF = 0.0;

    /*Swerve Drive Current Limit Config*/
    constexpr bool DriveEnableCurrentLimit = true;
    constexpr int DriveContinuousCurrentLimit = 20;
    constexpr int DrivePeakCurrentLimit = 40;
    constexpr double DrivePeakCurrentDuration = 0.1;

    /*Motor Inverts Config*/
    constexpr bool AngleMotorInvert = true;
    constexpr bool DriveMotorInvert = false;

    /* Swerve Profiling values */
    constexpr units::meters_per_second_t MaxSpeed{4.2};
    constexpr units::degrees_per_second_t MaxAngularVelocity{360 * 1.25};
    constexpr bool IsFieldRelative = true;
    constexpr bool IsOpenLoop = false;  
}

namespace Orientation{


    constexpr int FRONT = 0;
    constexpr int FRONT_RIGHT = 45;
    constexpr int RIGHT = 90;
    constexpr int DOWN_RIGHT = 135;
    constexpr int DOWN = 180;
    constexpr int DOWN_LEFT = 225;
    constexpr int LEFT = 270;
    constexpr int FRONT_LEFT = 315;
    constexpr int NON_ORIENTED = -1;
};

namespace ChargingStation{
    
    // const units::meter_t FrontLeft[2] = { 190.08_in,  154.94_in};
    // const units::meter_t FrontRight[2] = { 190.08_in, 63.43_in};
    // const units::meter_t BackLeft[2] = {115.22_in, 154.94_in};
    // const units::meter_t BackRight[2] = {115.22_in, 63.43_in};

    const std::pair<units::meter_t, units::meter_t> FrontLeft = std::make_pair<>(200.08_in, 164.94_in);
    const std::pair<units::meter_t, units::meter_t> FrontRight = std::make_pair<>(200.08_in, 53.43_in);
    const std::pair<units::meter_t, units::meter_t> BackLeft = std::make_pair<>(105.22_in, 164.94_in);
    const std::pair<units::meter_t, units::meter_t> BackRight = std::make_pair<>(105.22_in, 53.43_in);
    
}

namespace FrontLeftModule{
    constexpr int DriveMotorID = 0;
    constexpr int AngleMotorID = 1;
    constexpr int CanCoderID = 2;
    constexpr double AngleOffset = 352.617;
    const double Constants[4] = { DriveMotorID, AngleMotorID, CanCoderID, AngleOffset };
}

namespace FrontRightModule{
    constexpr int DriveMotorID = 3;
    constexpr int AngleMotorID = 4;
    constexpr int CanCoderID = 5;
    constexpr double AngleOffset = 321.855;
    const double Constants[4] = { DriveMotorID, AngleMotorID, CanCoderID, AngleOffset};
}
namespace BackLeftModule{
    constexpr int DriveMotorID = 6;
    constexpr int AngleMotorID = 7;
    constexpr int CanCoderID = 8;
    // constexpr auto AngleOffset = 266.045;
    constexpr double AngleOffset = 97.822;
    constexpr double Constants[4] = { DriveMotorID, AngleMotorID, CanCoderID, AngleOffset};
}
namespace BackRightModule{
    constexpr int DriveMotorID = 9;
    constexpr int AngleMotorID = 10;
    constexpr int CanCoderID = 11;
    constexpr double AngleOffset = 71.104;
    const double Constants[4] = { DriveMotorID, AngleMotorID, CanCoderID, AngleOffset};
}

namespace AutoConstants{
    constexpr units::meters_per_second_t MaxSpeed{ 4 };
    constexpr units::meters_per_second_squared_t MaxAccel{ 3 };
    constexpr units::radians_per_second_t MaxAngularSpeed{ 6 };
    constexpr units::radians_per_second_squared_t MaxAngularAccel{ 3 };

    /*Auto Swerve Drive Motor PID gains*/
    constexpr double XDriveKP = 5.0;
    constexpr double XDriveKD = 0.0;

    const frc2::PIDController XPID{ XDriveKP, 0.0, XDriveKD };
    
    constexpr double YDriveKP = 5.0;
    constexpr double YDriveKD = 0.0;
    
    const frc2::PIDController YPID{ YDriveKP, 0.0, YDriveKD };

    /* Auto Swerve Angle Motor PID gains*/  
    constexpr double AngleKP = 5.5;
    constexpr double AngleKD = 0.0;

    const frc2::PIDController ZPID{ YDriveKP, 0.0, AngleKD };

    const frc::ProfiledPIDController<units::angle::radian> ThetaPID{AngleKP, 
                                              0.0, 
                                              AngleKD, 
                                              frc::TrapezoidProfile<units::radians>::Constraints{MaxAngularSpeed,
                                                                                                 MaxAngularAccel}};
}
namespace IntakeConstants {

constexpr int kDriverControllerPort = 0;
constexpr int IntakePistonLA = 0;
constexpr int IntakePistonLB = 0;
constexpr int IntakePistonRA = 0;
constexpr int IntakePistonRB = 0;

}  // namespace OperatorConstants
