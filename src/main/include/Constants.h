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
#include <frc/controller/ArmFeedforward.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/trajectory/TrapezoidProfile.h>
#include <frc/controller/PIDController.h>
#include <frc/trajectory/TrapezoidProfile.h>


#include <frc/util/Color.h>

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
    const int ElbowBrake1 = 2;
    const int ElbowBrake2 = 3;
    const int ShoulderBrake1 = 4;
    const int ShoulderBrake2 = 5;
    const int ElbowAbsEncoderID = 0;
    const int WristAbsEncoderID = 1;
    //Shoulder constants
    const double kShoulderP = 0.7; 
    const double kShoulderI = 0.008;
    const double kShoulderD = 0.0;
    const double kShoulderUpP = 1.8;
    const double kShoulderUpI = 0.008;
    const double kShoulderUpD = 0.0;
    const int kShoulderIZone = 1.0; 
    const int kShoulderSlot = 0;

    //Elbow constants
    const double kElbowUpP = 0.01; 
    const double kElbowUpI = 0.000;
    const double kElbowUpD = 0.0; 
    const int kElbowUpSlot = 0;
    const double kElbowDownP = 0.01;
    const double kElbowDownI = 0.000;
    const double kElbowDownD = 0.0; 
    const int kElbowDownSlot = 1;
    const double kElbowIZone = 1.0;

    //Wrist constants
    const double kWristP = 1.0;
    const double kWristI = 0.0;
    const double kWristD = 0.0;

    //FF gains
    constexpr units::volt_t kElbowS{0.087205};
    constexpr units::volt_t kElbowG{0.0}; // keep this 0 according to alexander
    constexpr units::volt_t kElbowVolts{3.1139};
    constexpr units::volt_t kElbowVoltsAccel{0.068769};
    constexpr units::second_t kElbowT{1.0};
    constexpr units::radian_t kElbowR{1.0};

    constexpr auto kElbowV = (kElbowVolts * kElbowT) / kElbowR; 
    constexpr auto kElbowA = (kElbowVoltsAccel * kElbowT * kElbowT) / kElbowR;

    constexpr units::volt_t kShoulderS{0.084884};
    constexpr units::volt_t kShoulderG{0.0}; // keep this 0 according to alexander
    constexpr units::volt_t kShoulderVolts{3.293};
    constexpr units::volt_t kShoulderVoltsAccel{0.069236};
    constexpr units::second_t kShoulderT{1.0};
    constexpr units::radian_t kShoulderR{1.0};

    constexpr auto kShoulderV = (kShoulderVolts * kShoulderT) / kShoulderR; 
    constexpr auto kShoulderA = (kShoulderVoltsAccel * kShoulderT * kShoulderT) / kShoulderR;

    constexpr units::volt_t kWristS{0.15251};
    constexpr units::volt_t kWristG{0.0}; // keep this 0 according to alexander
    constexpr units::volt_t kWristVolts{1.6219};
    constexpr units::volt_t kWristVoltsAccel{0.028411};
    constexpr units::second_t kWristT{1.0};
    constexpr units::radian_t kWristR{1.0};

    constexpr auto kWristV = (kWristVolts * kWristT) / kWristR; 
    constexpr auto kWristA = (kWristVoltsAccel * kWristT * kWristT) / kWristR;

    constexpr units::angular_velocity::radians_per_second_t kEndVel{0};

    const double RotToRad = 0.03888641297;
    // SmartMotion constants (currently not being used)
    const double kElbowSmartMotionP = 0.0001;
    const double kElbowSmartMotionD = 0.0;
    const double kElbowSmartMotionFF = 0.03;
    const double kElbowMaxA = 80.0; 
    const double kElbowMaxV = 80.0; 
    const double kElbowMinV = 0.0;
    const double kShoulderMaxA = 25.0; 
    const double kShoulderMaxV = 2500.0; 
    const double kShoulderMinV = 0.0; 

    const int UpwardElbowCurrentLimit = 15;
    const int DownwardElbowCurrentLimit = 8;

    const double intakeAngleConversion = 0.0;  
    const double DegreesToRotations = 0.4423; 
    const double radiansToEncoder = 0.0; 
    const double AbsToRel = 16384/507; 
    const double JoystickToArm = 1.0; 
    const double TriggerToArm = 0.5;

    const double kWristStep = 1.0;
    const double kElbowStep = 0.5;
    const double kShoulderStep = 0.5;

    const double kElbowCrawl = 0.25;
    const double kShoulderCrawl = 0.25;
    const double kWristCrawl = 0.25;
    const double ErrorBound = 3;

    const double kElbowGearRatio = 161.58;

    //units are in meters
    const double ShoulderJointLength = 0.965;
    const double ElbowJointLength = 0.8507;
    const double IntakeJointLength = 0.457;

    //units are in kg
    const double ShoulderJointMass = 3.896;
    const double ElbowJointMass = 2.22;
    const double IntakeJointMass = 1.814;

    const double bufferZone = 2; 
    constexpr units::time::second_t ManualTimer{0.1};

    constexpr double HighConeShoulder = 16.285;
    constexpr double HighConeElbow = 25.854;
    constexpr double HighConeTilt = 9.432;

    constexpr double MidConeShoulder = 5.5;
    constexpr double MidConeElbow = 15.5;
    constexpr double MidConeTilt = 4.0;

    constexpr double HighCubeShoulder = 13.857;
    constexpr double HighCubeElbow = 21.484;
    constexpr double HighCubeTilt = 4.3;

    constexpr double MidCubeShoulder = 3.98;
    constexpr double MidCubeTempElbow = 16.6;
    constexpr double MidCubeTempElbowThreshold = 16;
    constexpr double MidCubeElbow = 12.0;
    constexpr double MidCubeTilt = 3.0;
    constexpr double MidCubeTiltThreshold = -5;

    constexpr double StowShoulder = 0.5;
    constexpr double StowTempElbow = 7.5;
    constexpr double StowCubeElbow = 1.5;
    constexpr double StowTempElbowThreshold = 20;
    constexpr double StowElbow = 1.5;
    constexpr double StowTilt = -20.925;
    constexpr double StowMidCubeTiltThreshold = -17;
    constexpr double StowHighCubeTiltThreshold = -14;
    constexpr double StowTiltThreshold = -15;
   
    constexpr double PortalElbow = 0.5;
    constexpr double PortalTilt = 10;
    constexpr double PortalTiltThreshold = -15;

    constexpr double TiltedStowShoulder = 0.05;
    constexpr double TiltedStowElbow = -6.95;
    constexpr double TiltedStowTilt = 6.0;
     
    constexpr double SubstationX = 0.0;
    constexpr double SubstationY = 0.0;
    constexpr double SubstationShoulder = 0.0;
    constexpr double SubstationElbow = 25.5;
    constexpr double SubstationElbowThreshold = 10;
    constexpr double SubstationTilt = 5.0;

    constexpr double GroundX = 0.0;
    constexpr double GroundY = 0.0;
    constexpr double GroundTempElbow = 8.0;
    constexpr double GroundTempElbowThreshold = 5.88;
    constexpr double GroundElbow = -4.75; 
    constexpr double GroundShoulder = 13.3;
    constexpr double TempElbow = 6.0;
    constexpr double GroundTilt = -25.0; 
    constexpr double GroundTiltThreshold = -10.5;

    constexpr double floorCubeShoulder = 0.5;
    constexpr double floorCubeElbow = 0.5;
    constexpr double floorCubeTilt = -18.0;

    constexpr double ScoreTilt = -6.0;
    constexpr double ScoreTimeout = 0.2;
}

namespace ControllerConstants{
    constexpr double Deadband = 0.1;
    constexpr double TriggerActivate = 0.8;
    
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
    constexpr double PitchKP = 0.1;
    constexpr double PitchKD = 0.0;


    //Tip Correction PID (ROLL) 
    constexpr double RollKP = 0.1;
    constexpr double RollKD = 0.0;

    constexpr units::second_t Timeout {0.1};
    

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
    constexpr int DriveContinuousCurrentLimit = 30;
    constexpr int DrivePeakCurrentLimit = 40;
    constexpr double DrivePeakCurrentDuration = 0.1;

    /*Motor Inverts Config*/
    constexpr bool AngleMotorInvert = true;
    constexpr bool DriveMotorInvert = false;

    /* Swerve Profiling values */
    constexpr units::meters_per_second_t MaxSpeed{5.0};
    constexpr units::degrees_per_second_t MaxAngularVelocity{360 * 1.25};
    constexpr bool IsFieldRelative = true;
    constexpr bool IsOpenLoop = false;  

}

namespace LEDConstants{
    enum LEDState{
        Standby, Yellow, Purple, Intaked, Error
    };
    constexpr units::second_t Timeout{ 10.0 };
    
    constexpr int CANdleID = 20;

}

namespace DPAD{

    enum ORIENTATION{
        FRONT = 0,
        FRONT_RIGHT = 45,
        RIGHT = 90,
        DOWN_RIGHT = 135,
        DOWN = 180,
        DOWN_LEFT,
        LEFT = 270,
        FRONT_LEFT = 315,
        NON_ORIENTED = -1
    };

    enum NODE_LEVEL{
        HIGH = 0,
        MID = 270,
        LOW = 180,
        NON_SPECIFIED = -1
    };
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
    constexpr units::meters_per_second_t MaxSpeed{ 2};
    constexpr units::meters_per_second_squared_t MaxAccel{ 2 };
    constexpr units::radians_per_second_t MaxAngularSpeed{ 12 };
    constexpr units::radians_per_second_squared_t MaxAngularAccel{ 3 };

    /*Auto Swerve Drive Motor PID gains*/
    constexpr double XDriveKP = 3.0;
    constexpr double XDriveKD = 0.0;

    const frc2::PIDController XPID{ XDriveKP, 0.0, XDriveKD };
    
    constexpr double YDriveKP = 3.0;
    constexpr double YDriveKD = 0.0;
    
    const frc2::PIDController YPID{ YDriveKP, 0.0, YDriveKD };

    /* Auto Swerve Angle Motor PID gains*/  
    constexpr double AngleKP = 3;
    constexpr double AngleKD = 0.0;

    const frc2::PIDController ZPID{ AngleKP, 0.0, AngleKD };

    const frc::ProfiledPIDController<units::angle::radian> ThetaPID{AngleKP, 
                                              0.0, 
                                              AngleKD, 
                                              frc::TrapezoidProfile<units::radians>::Constraints{MaxAngularSpeed,
                                                                                                 MaxAngularAccel}};
}



namespace IntakeConstants {
    // Intake Motor
    constexpr int IntakeMotorID = 15;
    constexpr double OuttakePower = 0.2;
    constexpr double IntakePower = -0.2;
    constexpr double PassivePower = -0.10;
    // PID stuff
    constexpr double kIntakeP = 0.1;
    constexpr double kIntakeI = 0.0; 
    constexpr double kIntakeD = 1.0;
    constexpr double kIntakeFF = 0.00025 / 1.6;
    // Constant for amount of time in transitioning between wheels on and toggle clamp
    constexpr units::second_t TimerConstant {0.2};
    
    constexpr int IntakePistonA = 0;
    constexpr int IntakePistonB = 1;

    // Conversion rate for velocity based on proximity
    constexpr double ProxToVelocity = 50.0;

    constexpr int CurrentLimit = 10;
    constexpr double MaxVelocity = 30.0;
    constexpr double kProximityP = 0.005;
    constexpr double kProximityD = 0.0001;

    constexpr units::second_t VibrationTimeout{2.5};

    constexpr units::meter_t CenterToStowedIntake{0.0};
    constexpr units::meter_t DistanceToMidCube{30.45_in};
    constexpr units::meter_t DistanceToHighCube{13.76_in};

    enum State{
        Purple = 0,
        Yellow =1,
        Empty = 2
    };

    enum IntakeMode{
        Off,
        Passive,
        Intake,
        Outake,
        MidShoot,
        HighShoot
    };
}



namespace ColorConstants {
    constexpr frc::Color PurpleTarget = frc::Color(0.202, 0.333, 0.459);
    constexpr frc::Color YellowTarget = frc::Color(0.361, 0.524, 0.113);

    constexpr double RecognitionProximity = 100.0;
    constexpr double TargetProximity = 20.0;
}

namespace GroundTakeConstants{
    constexpr int LeftBeamBreakID = 0;
    constexpr int CenterBeamBreakID = 1;
    constexpr int RightBeamBreakID = 2;


    constexpr int ExtenderID = 41;
    constexpr int IntakeID = 40;

    constexpr double kP = 0.0002;
    constexpr double kD = 0.00028;
    constexpr double kFF = 0.0007;
    constexpr double maxVel = 1280.0;
    constexpr double maxAccel = 1280.0;

    constexpr double ExtendedPosition = 6.19;
    constexpr double RetractedPosition = 0.0;

}