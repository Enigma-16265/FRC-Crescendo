package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class FlyWheelCommand extends Command 
{
    
    private final Shooter manipulator;
    private final Supplier<Double> speedSupplier;

    public FlyWheelCommand(
        Shooter shooter,
        Supplier<Double> speedSupplier )
    {
        this.manipulator = shooter;
        this.speedSupplier = speedSupplier;

        addRequirements(shooter);
    }

    @Override
    public void initialize() {}
    
    @Override
    public void execute()
    {

        double speed = speedSupplier.get();

        manipulator.spin(speed);

    }

    @Override
    public void end( boolean interrupted ) {}

    @Override
    public boolean isFinished() {
        return false;
    }

}
