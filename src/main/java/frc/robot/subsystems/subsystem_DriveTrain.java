// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.WPI_Pigeon2;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N3;
// import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.SwerveModule;
import frc.robot.Constants.DPAD;
import frc.robot.Constants.FrontLeftModule;
import frc.robot.Constants.FrontRightModule;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.BackLeftModule;
import frc.robot.Constants.BackRightModule;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.DPAD.ORIENTATION;
import frc.robot.Constants.SwerveConstants.Throttle;

public class subsystem_DriveTrain extends SubsystemBase {
  /** Creates a new subsystem_DriveTrain. */
  private WPI_Pigeon2 m_Gyro;

  private SwerveModule m_FrontLeft;
  private SwerveModule m_FrontRight;
  private SwerveModule m_BackLeft;
  private SwerveModule m_BackRight;
  
  private SwerveDriveKinematics m_Kinematics;
  private SwerveModulePosition[] m_ModulePositions;
  private SwerveDrivePoseEstimator m_PoseEstimator;
  
  private SwerveConstants.Throttle m_Throttle;

  private PIDController m_PitchCorrectionPID;
  private PIDController m_RollCorrectionPID;
  private PIDController m_AutoOrientPID;
  
  private boolean m_IsBalancing;
  private boolean m_IsAutoOrient;
  // private boolean m_IsPark;
  
  private int m_DPAD;
  private int m_OrientCounter;

  private Vector<N3> vec1 = VecBuilder.fill(0.7, 0.7, 0.1);
  private Vector<N3> vec2 = VecBuilder.fill(0.3, 0.3, 0.9);

  // private TrapezoidProfile.Constraints m_Constraints;

  public subsystem_DriveTrain() {
    m_Gyro = new WPI_Pigeon2(SwerveConstants.canCoderID, "DriveCANivore");
    m_FrontLeft = new SwerveModule(FrontLeftModule.constants);
    m_FrontRight = new SwerveModule(FrontRightModule.constants);
    m_BackLeft = new SwerveModule(BackLeftModule.constants);
    m_BackRight = new SwerveModule(BackRightModule.constants);
    m_Kinematics = new SwerveDriveKinematics(SwerveConstants.frontLeft, 
                                            SwerveConstants.frontRight,
                                            SwerveConstants.backLeft,
                                            SwerveConstants.backRight);

    m_ModulePositions = new SwerveModulePosition[4];
    updateModulePositions();
    
    m_PoseEstimator = new SwerveDrivePoseEstimator(m_Kinematics, 
                                                  // Rotation2d.fromDegrees(0.0),
                                                  new Rotation2d(),
                                                  m_ModulePositions,
                                                  new Pose2d(0.0, 0.0, 
                                                  // Rotation2d.fromDegrees(0.0)
                                                  new Rotation2d()
                                                  ), vec1, vec2);
    m_PitchCorrectionPID = new PIDController(0.0, 0.0, 0.0);
    m_RollCorrectionPID = new PIDController(0.01, 0.0, 0.0);

    m_Throttle = Throttle.LINEAR;

    m_IsBalancing = false;
    m_IsAutoOrient = false;
    // m_IsPark = false;
    m_OrientCounter = 0;
    m_DPAD = DPAD.value(ORIENTATION.NON_ORIENTED);
    m_AutoOrientPID = new PIDController(0.05, 0.0, 0.001);
    // m_Constraints = new TrapezoidProfile.Constraints(AutoConstants.MaxAngularSpeedMetersPerSecond, 
    //                                                 AutoConstants.MaxAngularAccelMetersPerSecondSquared);
    
    Timer.delay(1.0);
    resetModulesToAbsolute();
    m_Gyro.configFactoryDefault();
    zeroGyro();
    m_PitchCorrectionPID.setSetpoint(0.0);
    m_PitchCorrectionPID.setTolerance(2.0);

    m_RollCorrectionPID.setSetpoint(0.0);
    m_RollCorrectionPID.setTolerance(2.0);

    m_AutoOrientPID.enableContinuousInput(-180.0, 180.0);
    m_AutoOrientPID.setTolerance(1.0);

  }
  
  public void updateModulePositions(){
    m_ModulePositions[0] = m_FrontLeft.getPosition();
    m_ModulePositions[1] = m_FrontRight.getPosition();
    m_ModulePositions[2] = m_BackLeft.getPosition();
    m_ModulePositions[3] = m_BackRight.getPosition();
  }

  public boolean isBalanced(){
    return Math.abs(m_Gyro.getPitch()) < 2 && Math.abs(m_Gyro.getRoll()) < 2;
  }

  public void swerveDrive(double xSpeedMetersPerSecond, 
                          double ySpeedMetersPerSecond, 
                          double zRotRadiansPerSecond, 
                          boolean isFieldRelative, 
                          boolean isOpenLoop){
  if (m_IsBalancing){
    xSpeedMetersPerSecond += addRollCorrection() * getPoseYaw().getCos();
  }
  zRotRadiansPerSecond = m_IsAutoOrient ? getAngularVelocity() : zRotRadiansPerSecond;

  SwerveModuleState[] moduleStates = SwerveConstants.kinematics.toSwerveModuleStates(
    // isFieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
    //                       xSpeedMetersPerSecond,
    //                       ySpeedMetersPerSecond,
    //                       zRotRadiansPerSecond,
    //                       getPoseYaw()
    //                       ) : 
    //                   new ChassisSpeeds(xSpeedMetersPerSecond, ySpeedMetersPerSecond, zRotRadiansPerSecond)
    ChassisSpeeds.fromFieldRelativeSpeeds(
                          xSpeedMetersPerSecond,
                          ySpeedMetersPerSecond,
                          zRotRadiansPerSecond,
                          getPoseYaw()
                          )
  );
  SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, SwerveConstants.maxSpeed);
  // SwerveConstants.kinematics.desaturateWheelSpeeds(moduleStates, SwerveConstants.maxSpeed);
  SwerveModuleState frontLeft = moduleStates[0];
  SwerveModuleState frontRight = moduleStates[1];
  SwerveModuleState backLeft = moduleStates[2];
  SwerveModuleState backRight = moduleStates[3];

  m_FrontLeft.setDesiredState(frontLeft, isOpenLoop);
  m_FrontRight.setDesiredState(frontRight, isOpenLoop);
  m_BackLeft.setDesiredState(backLeft, isOpenLoop);
  m_BackRight.setDesiredState(backRight, isOpenLoop);

  }

  public void setModuleStates(SwerveModuleState[] desiredStates){
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, AutoConstants.MaxSpeedMetersPerSecond);
    // SwerveConstants.kinematics.desaturateWheelSpeeds(desiredStates, AutoConstants.MaxSpeedMetersPerSecond);
    m_FrontLeft.setDesiredState(desiredStates[0], false);
    m_FrontRight.setDesiredState(desiredStates[1], false);
    m_BackLeft.setDesiredState(desiredStates[2], false);
    m_BackRight.setDesiredState(desiredStates[3], false);
  }

  public double setThrottle(double input){
    return m_Throttle != Throttle.NONLINEAR ? input : Math.signum(input) * (1.01 * Math.pow(input, 2) - 0.0202 * input + 0.0101);
  }

  public void changeThrottle(){
    if(m_Throttle == Throttle.LINEAR){
      m_Throttle = Throttle.NONLINEAR;
      SmartDashboard.putBoolean("DRIVE MODE", false);
    } else {
      m_Throttle = Throttle.NONLINEAR;
      SmartDashboard.putBoolean("DRIVE MODE", true);
    }
  }

  public CommandBase toggleThrottleCommand(){
    return this.runOnce(() -> changeThrottle());
  }

  public void resetOdometry(Pose2d pose){
    updateModulePositions();
    m_PoseEstimator.resetPosition(m_Gyro.getRotation2d(), m_ModulePositions, pose);
  }

  public Pose2d getPose(){
    return m_PoseEstimator.getEstimatedPosition();
  }

  public Rotation2d getYaw(){
    double yaw = SwerveConstants.invertGyro ? 360.0 - m_Gyro.getYaw() : m_Gyro.getYaw();
    return Rotation2d.fromDegrees(yaw);
  }

  public Rotation2d getPoseYaw(){
    return m_PoseEstimator.getEstimatedPosition().getRotation();
  }

  public void toggleBalanceCorrection(){
    m_IsBalancing = !m_IsBalancing;
  }

  public void enableBalanceCorrection(){
    m_IsBalancing = true;
  }

  public void disableBalanceCorrection(){
    m_IsBalancing = false;
  }

  public void zeroGyro(){
    m_Gyro.setYaw(0.0);
  }

  public CommandBase zeroGyroCommand(){
    return this.runOnce(() -> zeroGyro());
  }

  public void resetModulesToAbsolute(){
    m_FrontLeft.resetToAbsolute();
    m_FrontRight.resetToAbsolute();
    m_BackLeft.resetToAbsolute();
    m_BackRight.resetToAbsolute();
  }

  public double addPitchCorrection(){
    return m_PitchCorrectionPID.calculate(m_Gyro.getPitch());
  }

  public double addRollCorrection(){
    return m_RollCorrectionPID.calculate(m_Gyro.getRoll());
  }

  public void setAutoOrient(boolean isOrientFront, boolean isOrientBack, double rotVelocity){
    if((Math.abs(rotVelocity) <= ControllerConstants.deadband) && DriverStation.isTeleopEnabled()){
      if(isOrientFront){
        if(DriverStation.getAlliance() == DriverStation.Alliance.Blue){
          m_DPAD = DPAD.value(ORIENTATION.RIGHT);
        } else {
          m_DPAD = DPAD.value(ORIENTATION.DOWN);
          m_IsAutoOrient = true;
        }
      }
    } else {
      m_IsAutoOrient = false;
      m_DPAD = DPAD.value(ORIENTATION.NON_ORIENTED);
    }
  }

  public double getAngularVelocity(){
    m_AutoOrientPID.setSetpoint(m_DPAD);
    if(m_AutoOrientPID.atSetpoint()){
      m_OrientCounter++;
      if(m_OrientCounter >= 4){
        m_IsAutoOrient = false;
        m_DPAD = DPAD.value(ORIENTATION.NON_ORIENTED);
        m_OrientCounter = 0;
        return 0.0;
      }
    }
    return m_AutoOrientPID.calculate(m_PoseEstimator.getEstimatedPosition().getRotation().getDegrees());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("X Position", m_PoseEstimator.getEstimatedPosition().getX());
    SmartDashboard.putNumber("Y Position", m_PoseEstimator.getEstimatedPosition().getY());
    SmartDashboard.putNumber("Pose Yaw", m_PoseEstimator.getEstimatedPosition().getRotation().getDegrees());
    m_PoseEstimator.update(m_Gyro.getRotation2d(), m_ModulePositions);
    
  }

}