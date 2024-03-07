package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorLift;
import frc.robot.subsystems.Shooter;

public class PivotCommand extends Command
{
    
    private final Shooter            manipulator;
    private final Supplier<Double>        speedSupplier;
    private final SlewRateLimiter         limiter;

    //Constants
    public final int kSpeedAccelerationLimit = 1;

    public PivotCommand(
        Shooter     manipulator,
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

        //double speed = speedSupplier.get();

        manipulator.setTargetAngle(45);

    }

    @Override
    public void end( boolean interrupted ) {}

    @Override
    public boolean isFinished() {
        return false;
    }

}
