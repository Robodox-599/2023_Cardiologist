// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.ControllerConstants;
// import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.commands.Autos;
// import frc.robot.commands.ExampleCommand;
import frc.robot.commands.command_DriveTeleop;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.subsystem_DriveTrain;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  // private final CommandXboxController m_driverController =
  //     new CommandXboxController(OperatorConstants.kDriverControllerPort);

  /* Controllers */
  private final Joystick driveRad = new Joystick(ControllerConstants.xboxDriveID);

  /* Drive Controls */
  private final int translationAxis = ControllerConstants.xboxLYAxis;
  private final int strafeAxis = ControllerConstants.xboxLXAxis;
  private final int rotationAxis = ControllerConstants.xboxRXAxis;

  private final int zeroGyroButton = ControllerConstants.xboxY;
  private final int changeThrottleButton = ControllerConstants.xboxRightJoyPress;

  /* Driver Buttons */
  private final JoystickButton zeroGyro = new JoystickButton(driveRad, zeroGyroButton);
  private final JoystickButton changeThrottle = new JoystickButton(driveRad, changeThrottleButton);

  /* Subsystems */
  private final subsystem_DriveTrain s_DriveTrain = new subsystem_DriveTrain();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    s_DriveTrain.setDefaultCommand(new command_DriveTeleop(s_DriveTrain, 
                                                          () -> {return -driveRad.getRawAxis(translationAxis);},
                                                          () -> {return -driveRad.getRawAxis(strafeAxis);},
                                                          () -> {return -driveRad.getRawAxis(rotationAxis);},
                                                          () -> {return driveRad.getRawButtonPressed(ControllerConstants.xboxLB);},
                                                          () -> {return driveRad.getRawButtonPressed(ControllerConstants.xboxRB);},
                                                          () -> {return SwerveConstants.isFieldRelative;},
                                                          () -> {return SwerveConstants.isOpenLoop;}));
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    // new Trigger(m_exampleSubsystem::exampleCondition)
    //     .onTrue(new ExampleCommand(m_exampleSubsystem));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    // m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());
    zeroGyro.onTrue(new InstantCommand(() -> s_DriveTrain.zeroGyroCommand()));
    changeThrottle.onTrue(new InstantCommand(() -> s_DriveTrain.toggleThrottleCommand()));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Autos.exampleAuto(m_exampleSubsystem);
  }
}
