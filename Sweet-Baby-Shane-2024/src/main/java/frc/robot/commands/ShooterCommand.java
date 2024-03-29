package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class ShooterCommand extends Command
{
    
    private final Shooter          manipulator;
    private final Supplier<Double> speedSupplier;

    //Constants
    public final int kSpeedAccelerationLimit = 1;

    public ShooterCommand(
        Shooter          manipulator,
        Supplier<Double> speedSupplier )
    {
        this.manipulator   = manipulator;
        this.speedSupplier = speedSupplier;

        addRequirements( manipulator );
    }

    @Override
    public void initialize() {}
    
    @Override
    public void execute()
    {

        double speed = speedSupplier.get();
        manipulator.spin( speed );

    }

    @Override
    public void end( boolean interrupted )
    {
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}
