// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.subsystem_DriveTrain;

public class command_DriveTeleop extends CommandBase {
  /** Creates a new command_DriveTeleop. */
  private subsystem_DriveTrain s_DriveTrain;
  private DoubleSupplier m_xSpeed;
  private DoubleSupplier m_ySpeed;
  private DoubleSupplier m_zRot;
  private BooleanSupplier m_IsOrientFront;
  private BooleanSupplier m_IsOrientBack;
  private BooleanSupplier m_FieldRelative;
  private BooleanSupplier m_OpenLoop;

  public command_DriveTeleop(subsystem_DriveTrain driveTrain,
                            DoubleSupplier xSpeed,
                            DoubleSupplier ySpeed,
                            DoubleSupplier zRot,
                            BooleanSupplier isOrientFront,
                            BooleanSupplier isOrientBack,
                            BooleanSupplier fieldRelative,
                            BooleanSupplier openLoop) {
    // Use addRequirements() here to declare subsystem dependencies.
    s_DriveTrain = driveTrain;
    m_xSpeed = xSpeed;
    m_ySpeed = ySpeed;
    m_zRot = zRot;
    m_IsOrientFront = isOrientFront;
    m_IsOrientBack = isOrientBack;
    m_FieldRelative = fieldRelative;
    m_OpenLoop = openLoop;
    addRequirements(s_DriveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    s_DriveTrain.setAutoOrient(m_IsOrientFront.getAsBoolean(), m_IsOrientBack.getAsBoolean(), m_zRot.getAsDouble());
    
    double transformedXSpeed = 0.0;
    double transformedYSpeed = 0.0;
    double transformedZRot = 0.0;

    if(Math.abs(m_xSpeed.getAsDouble()) > ControllerConstants.deadband){
      transformedXSpeed = s_DriveTrain.setThrottle(m_xSpeed.getAsDouble());
    }
    if(Math.abs(m_ySpeed.getAsDouble()) > ControllerConstants.deadband){
      transformedYSpeed = s_DriveTrain.setThrottle(m_ySpeed.getAsDouble());
    }
    if(Math.abs(m_zRot.getAsDouble()) > ControllerConstants.deadband){
      transformedZRot = s_DriveTrain.setThrottle(m_zRot.getAsDouble());
    }

    s_DriveTrain.swerveDrive(transformedXSpeed * SwerveConstants.maxSpeed,
                            transformedYSpeed * SwerveConstants.maxSpeed,
                            transformedZRot * SwerveConstants.maxAngularVelocity,
                            m_FieldRelative.getAsBoolean(),
                            m_OpenLoop.getAsBoolean());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
