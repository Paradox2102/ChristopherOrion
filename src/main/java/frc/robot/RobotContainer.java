// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.math.trajectory.constraint.TrajectoryConstraint;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.ArcadeDrive;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.ShooterCommand;
import frc.robot.commands.ThroatCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ThroatSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  public final DriveSubsystem m_driveSubsystem = new DriveSubsystem();
  private final ShooterSubsystem m_shooterSubsystem = new ShooterSubsystem();
  private final ThroatSubsystem m_throatSubsystem = new ThroatSubsystem();
  private final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();

  private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);

  private final Joystick m_joystick = new Joystick(0);

  JoystickButton m_shooter = new JoystickButton(m_joystick, 2);
  JoystickButton m_intake = new JoystickButton(m_joystick, 1);
  JoystickButton m_outtake = new JoystickButton(m_joystick, 8);
  JoystickButton m_throat = new JoystickButton(m_joystick, 3);

  TrajectoryConstraint m_autoConstraint;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    m_driveSubsystem.setDefaultCommand(new ArcadeDrive(m_driveSubsystem, () -> m_joystick.getX(), () -> m_joystick.getY(), () -> m_joystick.getThrottle()));
    m_throatSubsystem.setDefaultCommand(new ThroatCommand(m_throatSubsystem));
  
    m_shooter.toggleWhenPressed(new ShooterCommand(m_shooterSubsystem));
    m_intake.whileHeld(new IntakeCommand(m_intakeSubsystem, .75));
    m_outtake.whileHeld(new IntakeCommand(m_intakeSubsystem, -.75));
    m_throat.toggleWhenPressed(new ThroatCommand(m_throatSubsystem));

  }

  public Command getAutonomousCommand() {
    DriveSubsystem robotDrive = m_driveSubsystem;
    var autoVoltageConstraint = new DifferentialDriveVoltageConstraint(robotDrive.m_feedForward, m_driveSubsystem.k_DriveKinematics, robotDrive.k_volts);
    TrajectoryConfig config = new TrajectoryConfig(robotDrive.k_maxSpeedMetersPerSecond, robotDrive.k_maxAccelerationMetersPerSecondSquared).setKinematics(m_driveSubsystem.k_DriveKinematics).addConstraint(autoVoltageConstraint);//m_autoConstraint);

    Trajectory exampleTrajectory = 
    TrajectoryGenerator.generateTrajectory(
      new Pose2d(0, 0, new Rotation2d(0)), 
      List.of(new Translation2d(1, 1), new Translation2d(2, -1)), 
      new Pose2d(3, 0, new Rotation2d(0)),
      config);

    robotDrive.resetOdometry(exampleTrajectory.getInitialPose());

    RamseteCommand ramseteCommand = 
    new RamseteCommand(exampleTrajectory,
      robotDrive::getPose,
      new RamseteController(), // 2, .7
      robotDrive.m_feedForward,
      m_driveSubsystem.k_DriveKinematics,
      robotDrive::getWheelSpeeds,
      new PIDController(robotDrive.k_p, 0, 0),
      new PIDController(robotDrive.k_p, 0, 0),
      robotDrive::tankDriveVolts,
      robotDrive
      );
      SmartDashboard.putData(ramseteCommand);

      return ramseteCommand.andThen(() -> robotDrive.stop());
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {}

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
}
