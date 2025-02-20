// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.util.Color;
import frc.logging.DataNetworkTableLog;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.ShooterCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.IntakePivotHomingCommand;
import frc.robot.commands.IntakePivotCommand;
import frc.robot.commands.ElevatorCommand;
import frc.robot.commands.ElevatorHomingCommand;
import frc.robot.commands.ShooterPivotCommand;
import frc.robot.commands.ShooterPivotHomingCommand;
import frc.robot.commands.IntakePivotLimitCommand;
import frc.robot.commands.IntakeSlowReverseCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.IntakePivot;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterPivot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import java.util.List;
import java.util.Map;

import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();

    // The driver's controller
    XboxController m_driverController   = new XboxController(OIConstants.kDriverControllerPort);
    XboxController m_mechanicController = new XboxController(OIConstants.kMechanicControllerPort);

    static final boolean enableDriveTrain = true;

    // Subsystems
    private final Elevator     elevator     = new Elevator();
    private final Intake       intake       = new Intake();
    private final IntakePivot  intakePivot  = new IntakePivot();
    private final Shooter      shooter      = new Shooter();
    private final ShooterPivot shooterPivot = new ShooterPivot();

    // Commands
    // private final ElevatorCommand     elevatorCommand;
    private final IntakeCommand       intakeCommand;
    private final IntakePivotCommand  intakePivotCommand;
    private final ShooterCommand      shooterCommand;
    private final ShooterPivotCommand shooterPivotCommand;
    private final IntakeSlowReverseCommand   intakeSlowReverseCommand;

    // Driver Triggers
    private final Trigger m_driveLeftBumperTrigger;
    private final Trigger m_driverRightBumperTrigger;
    
    private final Trigger m_driverAButton;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    if ( enableDriveTrain )
    {
      // Configure default commands
      m_robotDrive.setDefaultCommand(
          // The left stick controls translation of the robot.
          // Turning is controlled by the X axis of the right stick.
          new RunCommand(
              () -> m_robotDrive.drive(
                  -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                  -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                  -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
                  true),
              m_robotDrive));
    }
            
    // elevatorCommand = new ElevatorCommand(
    //     elevator,
    //     () -> MathUtil.applyDeadband( -m_mechanicController.getLeftY(), OIConstants.kElevatorDeadband )
    // );
    // \elevator.setDefaultCommand(elevatorCommand);

    intakeCommand = new IntakeCommand( 
        intake,
        () -> {
          return ( MathUtil.applyDeadband( 
                    ( m_driverController.getLeftTriggerAxis() - m_driverController.getRightTriggerAxis() ),
                    OIConstants.kIntakeDeadband ) );
        },
        intakePivot
    );

    intake.setDefaultCommand(intakeCommand);

    intakePivotCommand = new IntakePivotCommand( 
        intakePivot,
        () -> {
          return ( MathUtil.applyDeadband( 
                    -( m_driverController.getLeftTriggerAxis() - m_driverController.getRightTriggerAxis() ),
                    OIConstants.kIntakePivotDeadband ) );
        }
    );

    // intakePivot.setDefaultCommand(intakePivotCommand);

    m_driveLeftBumperTrigger = new JoystickButton( m_driverController, XboxController.Button.kLeftBumper.value );
    m_driveLeftBumperTrigger.onTrue( new IntakePivotLimitCommand( intakePivot, IntakePivotLimitCommand.Behavior.GO ) );

    m_driverRightBumperTrigger = new JoystickButton( m_driverController, XboxController.Button.kRightBumper.value );
    m_driverRightBumperTrigger.onTrue( new IntakePivotLimitCommand( intakePivot, IntakePivotLimitCommand.Behavior.STOW ) );

    shooterCommand = new ShooterCommand(
        shooter,
        () -> {

          double speed = 0.0;

          if (m_mechanicController.getYButton() )
          {
            speed = 1.0;
          }
          else if ( m_mechanicController.getXButton() )
          {
            speed = 0.05;
          }
          else if ( m_mechanicController.getAButton() )
          {
            speed = -0.05;
          }

          return speed;

        }
    );
    shooter.setDefaultCommand(shooterCommand);

    shooterPivotCommand = new ShooterPivotCommand(
        shooterPivot,
        () -> MathUtil.applyDeadband( -m_mechanicController.getRightY(), OIConstants.kElevatorDeadband )
    );
    shooterPivot.setDefaultCommand( shooterPivotCommand );

    intakeSlowReverseCommand = new IntakeSlowReverseCommand( intake );

    m_driverAButton = new JoystickButton( m_driverController, XboxController.Button.kA.value );
    m_driverAButton.whileTrue( intakeSlowReverseCommand );
    
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    new JoystickButton(m_driverController, XboxController.Button.kX.value)
        .whileTrue(new RunCommand(
            () -> m_robotDrive.setX(),
            m_robotDrive));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Create config for trajectory
    TrajectoryConfig config = new TrajectoryConfig(
        AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(DriveConstants.kDriveKinematics);

    // An example trajectory to follow. All units in meters.
    Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(new Translation2d(0, 2), new Translation2d(0, -1)),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(0, 2, new Rotation2d(0)),
        config);

    var thetaController = new ProfiledPIDController(
        AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
        exampleTrajectory,
        m_robotDrive::getPose, // Functional interface to feed supplier
        DriveConstants.kDriveKinematics,

        // Position controllers
        new PIDController(AutoConstants.kPXController, 0, 0),
        new PIDController(AutoConstants.kPYController, 0, 0),
        thetaController,
        m_robotDrive::setModuleStates,
        m_robotDrive);

    // Reset odometry to the starting pose of the trajectory.
    m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());

    // Run path following command, then stop at the end.
    return swerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0, false));
  }

  public void subSystemReset()
  {
      Command elevatorHomingCommand = new ElevatorHomingCommand( elevator );
      // elevatorHomingCommand.schedule();

      Command intakePivotHomingCommand = new IntakePivotHomingCommand( intakePivot );
      // intakePivotHomingCommand.schedule();

      Command shooterPivotHomingCommand = new ShooterPivotHomingCommand( shooterPivot );
      //shooterPivotHomingCommand.schedule();
  }

}
