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
#include <frc/util/Color.h>

#include "FileReader.h"

using namespace std;

FileReader *fileReader = new FileReader();
map<string, string> robotParameters;
bool fileReadSuccess = fileReader->ReadConstantsFile(&robotParameters);

namespace ArmConstants {

    int PDH_ID  = stoi(robotParameters["PDH_ID"]);
    int ShoulderMotorID = stoi(robotParameters["ShoulderMotorID"]);
    int ShoulderFollowerID = stoi(robotParameters["ShoulderFollowerID"]);
    int ElbowMotorID = stoi(robotParameters["ElbowMotorID"]);
    int ElbowFollowerID = stoi(robotParameters["ElbowFollowerID"]);
    int WristMotorID = stoi(robotParameters["WristMotorID"]);
    int ElbowBrake1 = stoi(robotParameters["ElbowBrake1"]);
    int ElbowBrake2 = stoi(robotParameters["ElbowBrake2"]);
    int ShoulderBrake1 = stoi(robotParameters["ShoulderBrake1"]);
    int ShoulderBrake2 = stoi(robotParameters["ShoulderBrake2"]);
    
    //Shoulder constants
    double kShoulderP = stod(robotParameters["kShoulderP"]);
    double kShoulderI = stod(robotParameters["kShoulderI"]);
    double kShoulderD = stod(robotParameters["kShoulderD"]);
    double kShoulderUpP = stod(robotParameters["kShoulderUpP"]);
    double kShoulderUpI = stod(robotParameters["kShoulderUpI"]);
    double kShoulderUpD = stod(robotParameters["kShoulderUpD"]);
    int kShoulderIZone = stoi(robotParameters["kShoulderIZone"]); 
    int kShoulderSlot = stoi(robotParameters["kShoulderSlot"]);

    //Elbow constants
    double kElbowUpP = stod(robotParameters["kElbowUpP"]);
    double kElbowUpI = stod(robotParameters["kElbowUpI"]);
    double kElbowUpD = stod(robotParameters["kElbowUpD"]);
    int kElbowUpSlot = stoi(robotParameters["kElbowUpSlot"]);
    double kElbowDownP = stod(robotParameters["kElbowDownP"]);
    double kElbowDownI = stod(robotParameters["kElbowDownI"]);
    double kElbowDownD = stod(robotParameters["kElbowDownD"]);
    int kElbowDownSlot = stoi(robotParameters["kElbowDownSlot"]);
    double kElbowIZone = stod(robotParameters["kElbowIZone"]);

    //Wrist constants
    double kWristP = stod(robotParameters["kWristP"]);
    double kWristI = stod(robotParameters["kWristI"]);
    double kWristD = stod(robotParameters["kWristD"]);


    

    // SmartMotion constants (currently not being used)
    double kElbowSmartMotionP = stod(robotParameters["kElbowSmartMotionP"]);
    double kElbowSmartMotionD = stod(robotParameters["kElbowSmartMotionD"]);
    double kElbowSmartMotionFF = stod(robotParameters["kElbowSmartMotionFF"]);
    double kElbowMaxA = stod(robotParameters["kElbowMaxA"]);
    double kElbowMaxV = stod(robotParameters["kElbowMaxV"]);; 
    double kElbowMinV = stod(robotParameters["kElbowMinV"]);
    double kShoulderMaxA = stod(robotParameters["kShoulderMaxA"]); 
    double kShoulderMaxV = stod(robotParameters["kShoulderMaxV"]); 
    double kShoulderMinV = stod(robotParameters["kShoulderMinV"]);

    int UpwardElbowCurrentLimit = stoi(robotParameters["UpwardElbowCurrentLimit"]);
    int DownwardElbowCurrentLimit = stoi(robotParameters["DownwardElbowCurrentLimit"]);

    double intakeAngleConversion = stod(robotParameters["intakeAngleConversion"]);
    double DegreesToRotations = stod(robotParameters["DegreesToRotations"]);
    double radiansToEncoder = stod(robotParameters["radiansToEncoder"]);
    double AbsToRel = stod(robotParameters["AbsToRel"]); 
    double JoystickToArm = stod(robotParameters["JoystickToArm"]);
    double TriggerToArm = stod(robotParameters["TriggerToArm"]);

    double kWristStep = stod(robotParameters["kWristStep"]);
    double kElbowStep = stod(robotParameters["kElbowStep"]);
    double kShoulderStep = stod(robotParameters["kShoulderStep"]);

    //units are in meters
    double ShoulderJointLength = stod(robotParameters["ShoulderJointLength"]);
    double ElbowJointLength = stod(robotParameters["ElbowJointLength"]);
    double IntakeJointLength = stod(robotParameters["IntakeJointLength"]);

    //units are in kg
    double ShoulderJointMass = stod(robotParameters["ShoulderJointMass"]);
    double ElbowJointMass = stod(robotParameters["ElbowJointMass"]);
    double IntakeJointMass = stod(robotParameters["IntakeJointMass"]);

    double xOriginAdjustment = stod(robotParameters["xOriginAdjustment"]);
    double yOriginAdjustment = stod(robotParameters["yOriginAdjustment"]);

    double bufferZone = stod(robotParameters["bufferZone"]);
    constexpr units::time::second_t ManualTimer{0.1};

    double TicksOffset = stod(robotParameters["TicksOffset"]);
    double TicksToDegrees = stod(robotParameters["TicksToDegrees"]);

    double HighConeX = stod(robotParameters["HighConeX"]);
    double HighConeY = stod(robotParameters["HighConeY"]);
    double HighConeShoulder = stod(robotParameters["HighConeShoulder"]);
    double HighConeElbow = stod(robotParameters["HighConeElbow"]);
    double HighConeTilt = stod(robotParameters["HighConeTilt"]);

    double MidConeX = stod(robotParameters["MidConeX"]);
    double MidConeY = stod(robotParameters["MidConeY"]);
    double MidConeShoulder = stod(robotParameters["MidConeShoulder"]);
    double MidConeElbow = stod(robotParameters["MidConeElbow"]);
    double MidConeTilt = stod(robotParameters["MidConeTilt"]);

    double HighCubeX = stod(robotParameters["HighCubeX"]);
    double HighCubeY = stod(robotParameters["HighCubeY"]);
    double HighCubeShoulder = stod(robotParameters["HighCubeShoulder"]);
    double HighCubeElbow = stod(robotParameters["HighCubeElbow"]);
    double HighCubeTilt = stod(robotParameters["HighCubeTilt"]);

    double MidCubeX = stod(robotParameters["MidCubeX"]);
    double MidCubeY = stod(robotParameters["MidCubeY"]);
    double MidCubeShoulder = stod(robotParameters["MidCubeShoulder"]);
    double MidCubeElbow = stod(robotParameters["MidCubeElbow"]);
    double MidCubeTilt = stod(robotParameters["MidCubeTilt"]);

    double RestX = stod(robotParameters["RestX"]);
    double RestY = stod(robotParameters["RestY"]);
    double StowShoulder = stod(robotParameters["StowShoulder"]);
    double StowElbow = stod(robotParameters["StowElbow"]);
    double StowTilt = stod(robotParameters["StowTilt"]);

    double SubstationX = stod(robotParameters["SubstationX"]);
    double SubstationY = stod(robotParameters["SubstationY"]);
    double SubstationShoulder = stod(robotParameters["SubstationShoulder"]);
    double SubstationElbow = stod(robotParameters["SubstationElbow"]);
    double SubstationTilt = stod(robotParameters["SubstationTilt"]); 

    double GroundX = stod(robotParameters["GroundX"]);
    double GroundY = stod(robotParameters["GroundY"]);
    double GroundShoulder = stod(robotParameters["GroundShoulder"]);
    double GroundElbow = stod(robotParameters["GroundElbow"]);
    double TempElbow = stod(robotParameters["TempElbow"]);
    double GroundTilt = stod(robotParameters["GroundTilt"]);

    double floorCubeShoulder = stod(robotParameters["floorCubeShoulder"]);
    double floorCubeElbow = stod(robotParameters["floorCubeElbow"]);
    double floorCubeTilt = stod(robotParameters["floorCubeTilt"]);


    double ArmBackLimit = stod(robotParameters["ArmBackLimit"]);
    double ArmFrontLimit = stod(robotParameters["ArmFrontLimit"]);
}

namespace ControllerConstants{
    double Deadband = stod(robotParameters["Deadband"]);
    double TriggerActivate = stod(robotParameters["TriggerActivate"]);
    
    int XboxDriveID = stoi(robotParameters["XboxDriveID"]);
    int XboxYaperatorID = stoi(robotParameters["XboxYaperatorID"]);

    int xboxLXAxis = stoi(robotParameters["xboxLXAxis"]);
    int xboxLYAxis = stoi(robotParameters["xboxLYAxis"]);
    int xboxRXAxis = stoi(robotParameters["xboxRXAxis"]);
    int xboxRYAxis = stoi(robotParameters["xboxRYAxis"]);

    int xboxLTAxis = stoi(robotParameters["xboxLTAxis"]);
    int xboxRTAxis = stoi(robotParameters["xboxRTAxis"]);

    int xboxA = stoi(robotParameters["xboxA"]);
    int xboxB = stoi(robotParameters["xboxB"]);
    int xboxX = stoi(robotParameters["xboxX"]);
    int xboxY = stoi(robotParameters["xboxY"]);
    int xboxLB = stoi(robotParameters["xboxLB"]);
    int xboxRB = stoi(robotParameters["xboxRB"]);
    int xboxView = stoi(robotParameters["xboxView"]);
    int xboxMenu = stoi(robotParameters["xboxMenu"]);
    int xboxLeftJoyPress = stoi(robotParameters["xboxLeftJoyPress"]);
    int xboxRightJoyPress = stoi(robotParameters["xboxRightJoyPress"]);
    int xboxRightDPad = stoi(robotParameters["xboxRightDPad"]);
}


namespace SwerveConstants{

    int BalancekP = stoi(robotParameters["BalancekP"]);
    int BalancekI = stoi(robotParameters["BalancekI"]);
    int BalancekD = stoi(robotParameters["BalancekD"]);

    int CANCoderID = stoi(robotParameters["CANCoderID"]);
    bool InvertGyro = false;

    /*Drivetrain constants*/
    double OpenLoopRamp = stod(robotParameters["OpenLoopRamp"]);
    double closedLoopRamp = stod(robotParameters["closedLoopRamp"]);

    double DriveGearRatio = stod(robotParameters["DriveGearRatio"]);
    double AngleGearRatio = stod(robotParameters["AngleGearRatio"]);

    
    units::meter_t WheelCircumference{ 4.0_in * M_PI  };

    frc::Translation2d m_FrontLeft{11.375_in, 11.375_in};
    frc::Translation2d m_FrontRight{11.375_in, -11.375_in};
    frc::Translation2d m_BackLeft{-11.375_in, 11.375_in};
    frc::Translation2d m_BackRight{-11.375_in, -11.375_in};

    frc::SwerveDriveKinematics<4> m_kinematics{m_FrontLeft,
                                               m_FrontRight,
                                               m_BackLeft,
                                               m_BackRight};




    //Change to non-linear throttle for finer tuned movements
    enum Throttle {
        LINEAR = 1,
        NONLINEAR = 2
    };

    enum LEDState{
        Standby, Yellow, Purple
    };
    // units::second_t LEDTimeout{ 10.0 };



    //Tip Correction PID (PITCH)
    double PitchKP = stod(robotParameters["PitchKP"]);
    double PitchKD = stod(robotParameters["PitchKD"]);


    //Tip Correction PID (ROLL) 
    double RollKP = stod(robotParameters["RollKP"]);
    double RollKD = stod(robotParameters["RollKD"]);

    units::second_t Timeout {0.1};
    

    /*setting up correct units for the simepleMotorFeedforward KS gain*/
    units::volt_t DriveKS{0.1646};

    units::volt_t VoltageKV{2.239};
    units::foot_t FeetKV{1.0};
    units::second_t TimeKV{1.0};
    /*Setting up correct units for the simpleMotorFeedforward KV gain
    Change VoltageKV when wanting to change 
    the KV gain*/
    auto DriveKV = VoltageKV * TimeKV / FeetKV;

    units::volt_t VoltageKA{0.73308};
    units::foot_t FeetKA{1.0};
    units::second_t TimeKA{1.0};
    /*Setting up correct units for the simpleMotorFeedforward KA gain
    Change VoltageKA when wanting to change the KA gain*/
    auto DriveKA = VoltageKA * (TimeKA * TimeKA) / FeetKA; 
    units::volt_t kNominal {12.0};
    /*Angle Encoder Invert*/
    bool CanCoderInvert = false;

    /*Swerve Angle Motor PID gains*/
    double AngleKP = stod(robotParameters["AngleKP"]);
    double AngleKI = stod(robotParameters["AngleKI"]);
    double AngleKD = stod(robotParameters["AngleKD"]);
    double AngleKF = stod(robotParameters["AngleKF"]);

    /*Swerve Angle Current Limit Config*/
    bool AngleEnableCurrentLimit = true;
    int AngleContinuousCurrentLimit = stoi(robotParameters["AngleContinuousCurrentLimit"]);
    int AnglePeakCurrentLimit = stoi(robotParameters["AnglePeakCurrentLimit"]);
    double AnglePeakCurrentDuration = stod(robotParameters["AnglePeakCurrentDuration"]);
    
    /*Swerve Drive Motor PID gains*/
    double DriveKP = stod(robotParameters["DriveKP"]);
    double DriveKI = stod(robotParameters["DriveKI"]);
    double DriveKD = stod(robotParameters["DriveKD"]);
    double DriveKF = stod(robotParameters["DriveKF"]);

    /*Swerve Drive Current Limit Config*/
    bool DriveEnableCurrentLimit = true;
    int DriveContinuousCurrentLimit = stoi(robotParameters["DriveContinuousCurrentLimit"]);
    int DrivePeakCurrentLimit = stoi(robotParameters["DrivePeakCurrentLimit"]);
    double DrivePeakCurrentDuration = stod(robotParameters["DrivePeakCurrentDuration"]);

    /*Motor Inverts Config*/
    bool AngleMotorInvert = true;
    bool DriveMotorInvert = false;

    /* Swerve Profiling values */
    units::meters_per_second_t MaxSpeed{4.2};
    units::degrees_per_second_t MaxAngularVelocity{360 * 1.25};
    bool IsFieldRelative = true;
    bool IsOpenLoop = false;

    int CANdleID = stoi(robotParameters["CANdleID"]);
}

namespace Orientation{


    int FRONT = stoi(robotParameters["FRONT"]);
    int FRONT_RIGHT = stoi(robotParameters["FRONT_RIGHT"]);
    int RIGHT = stoi(robotParameters["RIGHT"]);
    int DOWN_RIGHT = stoi(robotParameters["DOWN_RIGHT"]);
    int DOWN = stoi(robotParameters["DOWN"]);
    int DOWN_LEFT = stoi(robotParameters["DOWN_LEFT"]);
    int LEFT = stoi(robotParameters["LEFT"]);
    int FRONT_LEFT = stoi(robotParameters["FRONT_LEFT"]);
    int NON_ORIENTED = stoi(robotParameters["NON_ORIENTED"]);
};

namespace ChargingStation{
    
    // units::meter_t FrontLeft[2] = { 190.08_in,  154.94_in};
    // units::meter_t FrontRight[2] = { 190.08_in, 63.43_in};
    // units::meter_t BackLeft[2] = {115.22_in, 154.94_in};
    // units::meter_t BackRight[2] = {115.22_in, 63.43_in};

    std::pair<units::meter_t, units::meter_t> FrontLeft = std::make_pair<>(200.08_in, 164.94_in);
    std::pair<units::meter_t, units::meter_t> FrontRight = std::make_pair<>(200.08_in, 53.43_in);
    std::pair<units::meter_t, units::meter_t> BackLeft = std::make_pair<>(105.22_in, 164.94_in);
    std::pair<units::meter_t, units::meter_t> BackRight = std::make_pair<>(105.22_in, 53.43_in);
    
}

namespace FrontLeftModule{
    int DriveMotorID = stoi(robotParameters["DriveMotorID"]);
    int AngleMotorID = stoi(robotParameters["AngleMotorID"]);
    int CanCoderID = stoi(robotParameters["CanCoderID"]);
    double AngleOffset = stod(robotParameters["AngleOffset"]);
    double Constants[4] = { DriveMotorID, AngleMotorID, CanCoderID, AngleOffset };
}

namespace FrontRightModule{
    int DriveMotorID = stoi(robotParameters["DriveMotorID"]);
    int AngleMotorID = stoi(robotParameters["AngleMotorID"]);
    int CanCoderID = stoi(robotParameters["CanCoderID"]);
    double AngleOffset = stod(robotParameters["AngleOffset"]);
    double Constants[4] = { DriveMotorID, AngleMotorID, CanCoderID, AngleOffset};
}
namespace BackLeftModule{
    int DriveMotorID = stoi(robotParameters["DriveMotorID"]);
    int AngleMotorID = stoi(robotParameters["AngleMotorID"]);
    int CanCoderID = stoi(robotParameters["CanCoderID"]);
    // auto AngleOffset = 266.045;
    double AngleOffset = stod(robotParameters["AngleOffset"]);
    double Constants[4] = { DriveMotorID, AngleMotorID, CanCoderID, AngleOffset};
}
namespace BackRightModule{
    int DriveMotorID = stoi(robotParameters["DriveMotorID"]);
    int AngleMotorID = stoi(robotParameters["AngleMotorID"]);
    int CanCoderID = stoi(robotParameters["CanCoderID"]);
    double AngleOffset = stod(robotParameters["AngleOffset"]);
    double Constants[4] = { DriveMotorID, AngleMotorID, CanCoderID, AngleOffset};
}

namespace AutoConstants{
    units::meters_per_second_t MaxSpeed{ 2 };
    units::meters_per_second_squared_t MaxAccel{ 1 };
    units::radians_per_second_t MaxAngularSpeed{ 6 };
    units::radians_per_second_squared_t MaxAngularAccel{ 3 };

    /*Auto Swerve Drive Motor PID gains*/
    double XDriveKP = stod(robotParameters["XDriveKP"]);
    double XDriveKD = stod(robotParameters["XDriveKD"]);

    frc2::PIDController XPID{ XDriveKP, 0.0, XDriveKD };
    
    double YDriveKP = stod(robotParameters["YDriveKP"]);
    double YDriveKD = stod(robotParameters["YDriveKD"]);
    
    frc2::PIDController YPID{ YDriveKP, 0.0, YDriveKD };

    /* Auto Swerve Angle Motor PID gains*/  
    double AngleKP = stod(robotParameters["AngleKP"]);
    double AngleKD = stod(robotParameters["AngleKD"]);

    frc2::PIDController ZPID{ YDriveKP, 0.0, AngleKD };

    frc::ProfiledPIDController<units::angle::radian> ThetaPID{AngleKP, 
                                              0.0, 
                                              AngleKD, 
                                              frc::TrapezoidProfile<units::radians>::Constraints{MaxAngularSpeed,
                                                                                                 MaxAngularAccel}};
}

namespace IntakeConstants {
    // Intake Motor
    int IntakeMotorID = stoi(robotParameters["IntakeMotorID"]);
    double OuttakePower = stod(robotParameters["OuttakePower"]);
    double IntakePower = stod(robotParameters["IntakePower"]);
    double PassivePower = stod(robotParameters["PassivePower"]);
    // PID stuff
    double kIntakeP = stod(robotParameters["kIntakeP"]);
    double kIntakeI = stod(robotParameters["kIntakeI"]);
    double kIntakeD = stod(robotParameters["kIntakeD"]);
    double kIntakeFF = stod(robotParameters["kIntakeFF"]);
    // Constant for amount of time in transitioning between wheels on and toggle clamp
    units::second_t TimerConstant {0.2};
    units::second_t VibrationTime {0.25};
    
    int IntakePistonA = stoi(robotParameters["IntakePistonA"]);
    int IntakePistonB = stoi(robotParameters["IntakePistonB"]);

    // Conversion rate for velocity based on proximity
    double ProxToVelocity = stod(robotParameters["ProxToVelocity"]);

    int CurrentLimit = stoi(robotParameters["CurrentLimit"]);
    double MaxVelocity = stod(robotParameters["MaxVelocity"]);
    double kProximityP = stod(robotParameters["kProximityP"]);
    double kProximityD = stod(robotParameters["kProximityD"]);

    units::meter_t CenterToStowedIntake{0.0};
    units::meter_t DistanceToMidCube{30.45_in};
    units::meter_t DistanceToHighCube{13.76_in};

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
    frc::Color PurpleTarget = frc::Color(0.202, 0.333, 0.459);
    frc::Color YellowTarget = frc::Color(0.361, 0.524, 0.113);

    double RecognitionProximity = stod(robotParameters["RecognitionProximity"]);
    double TargetProximity = stod(robotParameters["TargetProximity"]);
}