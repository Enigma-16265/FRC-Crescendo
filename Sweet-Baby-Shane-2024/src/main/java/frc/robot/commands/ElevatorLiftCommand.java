package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
// import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;

public class ElevatorLiftCommand extends Command
{

    private final Elevator            manipulator;
    private final Supplier<Double>        speedSupplier;
    private final SlewRateLimiter         limiter;

    //Constants
    public final int kSpeedAccelerationLimit = 1;

    public ElevatorLiftCommand(
        Elevator     manipulator,
        Supplier<Double> speedSupplier )
    {
        this.manipulator   = manipulator;
        this.speedSupplier = speedSupplier;
        
        limiter = new SlewRateLimiter( kSpeedAccelerationLimit );

        addRequirements( manipulator );
    }

    @Override
    public void initialize() {}
    
    @Override
    public void execute()
    {

        double speed = speedSupplier.get();

        manipulator.lift(speed);

    }

    @Override
    public void end( boolean interrupted ) {}

    @Override
    public boolean isFinished() {
        return false;
    }

}
