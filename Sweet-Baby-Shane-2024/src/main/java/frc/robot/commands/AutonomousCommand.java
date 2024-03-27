package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.IntakePivot;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.DriveSubsystem;

public class AutonomousCommand extends SequentialCommandGroup {

    public AutonomousCommand(Shooter shooter, Intake intake, DriveSubsystem driveTrain, IntakePivot intakePivot) {
        addCommands(
            // Spin the shooter at speed 1 for 1 second
            new ShooterCommand(shooter, () -> 1.0).withTimeout(1),

            // Wait for 1 second before setting intake speed to 1
            new WaitCommand(1).andThen(new IntakeCommand(intake, () -> { double speed = 1.0; return speed; }, intakePivot).withTimeout(1.5)),

            new RunCommand(() -> {
                shooter.stop(); // Assuming you have a method to stop the shooter
                intake.stop(); // Assuming you have a method to stop the intake
            }, shooter, intake),

            // Drive backwards for 2 seconds at 0.2 speed
            new RunCommand(() -> driveTrain.drive(-0.2, 0, 0, false, true), driveTrain).withTimeout(2),

            // Stop them all from moving
            new RunCommand(() -> driveTrain.drive(0, 0, 0, false, true), driveTrain),

            // Lower the intake and set intake speed to -0.8 for 0.3 seconds
            new IntakePivotLimitCommand( intakePivot, IntakePivotLimitCommand.Behavior.GO ).andThen(
                new IntakeCommand(intake, () -> { double speed = -0.8; return speed; }, intakePivot).withTimeout(0.3)
            ),

            // Call the intake to go upwards as shown in the robot container code
            new IntakePivotLimitCommand( intakePivot, IntakePivotLimitCommand.Behavior.STOW ),

            // Drive forward for 2 seconds at 0.2 speed
            new RunCommand(() -> driveTrain.drive(0.2, 0, 0, false, true), driveTrain).withTimeout(2),

            // Speed up the shooter again for 1 second
            new ShooterCommand(shooter, () -> 1.0).withTimeout(1),

            // Speed up the intake for 1.5 seconds
            new IntakeCommand(intake, () -> { double speed = 1.0; return speed; }, intakePivot).withTimeout(1.5),

            // Stop everything
            new RunCommand(() -> {
                driveTrain.drive(0, 0, 0, false, true);
                shooter.stop(); // Assuming you have a method to stop the shooter
                intake.stop(); // Assuming you have a method to stop the intake
            }, driveTrain, shooter, intake)
        );
    }
}
