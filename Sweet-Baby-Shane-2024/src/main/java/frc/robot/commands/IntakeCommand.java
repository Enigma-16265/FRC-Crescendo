package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorLift;
import frc.robot.subsystems.Intake;

public class IntakeCommand extends Command 
{
    
    private final Intake manipulator;
    private final Supplier<Double> speedSupplier;
    private final SlewRateLimiter limiter;

    // Constants
    public final int kSpeedAccelerationLimit = 1;

    public IntakeCommand(
        Intake intake,
        Supplier<Double> speedSupplier )
    {
        this.manipulator = intake;
        this.speedSupplier = speedSupplier;

        limiter = new SlewRateLimiter(kSpeedAccelerationLimit);

        addRequirements(intake);
    }

    @Override
    public void initialize() {}
    
    @Override
    public void execute()
    {

        double speed = speedSupplier.get();

        manipulator.intake(speed);

    }

    @Override
    public void end( boolean interrupted ) {}

    @Override
    public boolean isFinished() {
        return false;
    }

}
