// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *  
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class SwerveConstants{
    public static final int balancekP = 0;
    public static final int balancekI = 0;
    public static final int balancekD = 0;

    public static final int canCoderID = 12; 
    public static final boolean invertGyro = false;

    // Drivetrain Constants
    public static final double openLoopRamp = 0.25;
    public static final double closedLoopRamp = 0.375;

    public static final double driveGearRatio = 6.75;
    public static final double angleGearRatio = 150.0 / 7.0;

    
    public static final double wheelCircumference = Units.inchesToMeters(4.0 * Math.PI);

    public static final Translation2d frontLeft = new Translation2d(Units.inchesToMeters(11.375), Units.inchesToMeters(11.375));
    public static final Translation2d frontRight = new Translation2d(Units.inchesToMeters(11.375), -Units.inchesToMeters(11.375));
    public static final Translation2d backLeft = new Translation2d(-Units.inchesToMeters(11.375), Units.inchesToMeters(11.375));
    public static final Translation2d backRight = new Translation2d(-Units.inchesToMeters(11.375), -Units.inchesToMeters(11.375));
    
    public static final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(frontLeft, frontRight, backLeft, backRight);
    
    //Change to non-linear throttle for finer tuned movements
    public static enum Throttle{
      LINEAR,
      NONLINEAR
    };

    //Tip Correction PID (PITCH)
    public static final double pitchKP = 0.1;
    public static final double pitchKD = 0.0;

    //Tip Correction PID (ROLL) 
    public static final double rollKP = 0.1;
    public static final double rollKD = 0.0;
    public static final double timeout = 0.1;
    
    /*setting up correct units for the simepleMotorFeedforward KS gain*/
    public static final double driveKS = 0.1646;

    public static final double voltageKV = 2.239;
    public static final double feetKV = 1.0;
    public static final double timeKV = 1.0;
    /*Setting up correct units for the simpleMotorFeedforward KV gain
    Change VoltageKV when wanting to change 
    the KV gain*/
    public static final double driveKV = voltageKV * timeKV / feetKV;

    public static final double voltageKA = 0.73308;
    public static final double feetKA = 1.0;
    public static final double timeKA = 1.0;
    /*Setting up correct units for the simpleMotorFeedforward KA gain
    Change VoltageKA when wanting to change the KA gain*/
    public static final double driveKA = voltageKA * (timeKA * timeKA) / feetKA; 
    public static final double kNominal = 12.0;
    /*Angle Encoder Invert*/
    public static final boolean canCoderInvert = false;

    /*Swerve Angle Motor PID gains*/
    public static final double angleKP = 0.15;
    public static final double angleKI = 0.0;
    public static final double angleKD = 0.00;
    public static final double angleKF = 0.0;

    /*Swerve Angle Current Limit Config*/
    public static final boolean angleEnableCurrentLimit = true;
    public static final int angleContinuousCurrentLimit = 10;
    public static final int anglePeakCurrentLimit = 20;
    public static final double anglePeakCurrentDuration = 0.1;
    
    /*Swerve Drive Motor PID gains*/
    public static final double driveKP = 0.1;
    public static final double driveKI = 0.0;
    public static final double driveKD = 0.00;
    public static final double driveKF = 0.0;

    /*Swerve Drive Current Limit Config*/
    public static final boolean driveEnableCurrentLimit = true;
    public static final int driveContinuousCurrentLimit = 30;
    public static final int drivePeakCurrentLimit = 40;
    public static final double drivePeakCurrentDuration = 0.1;

    /*Motor Inverts Config*/
    public static final boolean angleMotorInvert = true;
    public static final boolean driveMotorInvert = false;

    /* Swerve Profiling values */
    public static final double maxSpeed = 5.0; // mps
    public static final double maxAngularVelocity = 360 * 0.25; // deg per sec
    public static final boolean isFieldRelative = true;
    public static final boolean isOpenLoop = false;  
  }

  public static final class DPAD{

    public static enum ORIENTATION{
        FRONT,
        FRONT_RIGHT,
        RIGHT,
        DOWN_RIGHT,
        DOWN,
        DOWN_LEFT,
        LEFT,
        FRONT_LEFT,
        NON_ORIENTED
    };

    public static enum NODE_LEVEL{
        HIGH,
        MID,
        LOW,
        NON_SPECIFIED
    };

    public static int value(ORIENTATION o){
      switch(o){
        case FRONT:        return 0;
        case FRONT_RIGHT:  return 45;
        case RIGHT:        return 90;
        case DOWN_RIGHT:   return 135;
        case DOWN:         return 180;
        case DOWN_LEFT:    return 0;
        case LEFT:         return 270;
        case FRONT_LEFT:   return 315;
        default:           return -1;
      }
    }
    
    // public int value(NODE_LEVEL n){
    //   switch(n){
    //     case HIGH: return 0;
    //     case MID:  return 270;
    //     case LOW:  return 180;
    //     default: return -1;
    //   }
    // }

  }

  public static final class FrontLeftModule{
    public static final int driveMotorID = 0;
    public static final int angleMotorID = 1;
    public static final int canCoderID = 2;
    public static final double angleOffset = 352.002; //meant to be negative
    public static final double[] constants = { driveMotorID, angleMotorID, canCoderID, angleOffset };
  }

  public static final class FrontRightModule{
    public static final int driveMotorID = 3;
    public static final int angleMotorID = 4;
    public static final int canCoderID = 5;
    public static final double angleOffset = 321.328;//meant to be negative
    public static final double[] constants = { driveMotorID, angleMotorID, canCoderID, angleOffset};
  }

  public static final class BackLeftModule{
    public static final int driveMotorID = 6;
    public static final int angleMotorID = 7;
    public static final int canCoderID = 8;
    public static final double angleOffset = 98.525;
    public static final double[] constants = { driveMotorID, angleMotorID, canCoderID, angleOffset};
  }

  public static final class BackRightModule{
    public static final int driveMotorID = 9;
    public static final int angleMotorID = 10;
    public static final int canCoderID = 11;
    public static final double angleOffset = 68.555;
    public static final double[] constants = { driveMotorID, angleMotorID, canCoderID, angleOffset};
  }

  public static class AutoConstants{
    public static final int MaxSpeedMetersPerSecond = 1;
    public static final int MaxAccelMetersPerSecondSquared = 1;
    public static final int MaxAngularSpeedMetersPerSecond = 6;
    public static final int MaxAngularAccelMetersPerSecondSquared = 3;

    /*Auto Swerve Drive Motor PID gains*/
    public static final double XDriveKP = 1.0;
    public static final double XDriveKD = 0.0;

    public static final PIDController XPID = new PIDController(XDriveKP, 0.0, XDriveKD);
    
    public static final double YDriveKP = 1.0;
    public static final double YDriveKD = 0.0;
    
    public static final PIDController YPID = new PIDController(YDriveKP, 0.0, YDriveKD);

    /* Auto Swerve Angle Motor PID gains*/  
    public static final double AngleKP = 3;
    public static final double AngleKD = 0.0;

    public static final PIDController ZPID = new PIDController (AngleKP, 0.0, AngleKD);

    public static final ProfiledPIDController ThetaPIDRadians = new ProfiledPIDController(
      AngleKP, 
      0.0, 
      AngleKD, 
      new TrapezoidProfile.Constraints(MaxAngularSpeedMetersPerSecond, MaxAngularAccelMetersPerSecondSquared));
}

  public static class ControllerConstants{
    public static final double deadband = 0.1;
    public static final double triggerActivate = 0.8;
    
    public static final int xboxDriveID = 0;
    public static final int xboxYaperatorID = 1;

    public static final int xboxLXAxis = 0;
    public static final int xboxLYAxis = 1;
    public static final int xboxRXAxis = 4;
    public static final int xboxRYAxis = 5;

    public static final int xboxLTAxis = 2;
    public static final int xboxRTAxis = 3;

    public static final int xboxA = 1;
    public static final int xboxB = 2;
    public static final int xboxX = 3;
    public static final int xboxY = 4;
    public static final int xboxLB = 5;
    public static final int xboxRB = 6;
    public static final int xboxView = 7;
    public static final int xboxMenu = 8;
    public static final int xboxLeftJoyPress = 9;
    public static final int xboxRightJoyPress = 10;
    public static final int xboxRightDPad = 11;
  }
}
