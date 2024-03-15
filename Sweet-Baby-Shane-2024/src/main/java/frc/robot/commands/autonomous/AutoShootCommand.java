package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.ShooterCommand;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.IntakePivot;
import frc.robot.subsystems.Shooter;

public class AutoShootCommand extends SequentialCommandGroup {
    public AutoShootCommand(Shooter shooter, Intake intake, IntakePivot intakePivot) {
        addCommands(
            // Command 1: ShooterCommand with speed 0.5
            new ShooterCommand(shooter, () -> 1.0),
            // Wait for 2 seconds
            new WaitCommand(1),
            // Command 2: IntakeCommand with speed 0.5
            new IntakeCommand(intake, () -> -0.4, intakePivot),
            // Wait for Another Second
            new WaitCommand(1),
            // Instantly stop the commands after the wait
            new InstantCommand
            (() -> {
                    shooter.stop();
                    intake.stop();
                    intakePivot.stop();
                }
            )
        );
    }
}
