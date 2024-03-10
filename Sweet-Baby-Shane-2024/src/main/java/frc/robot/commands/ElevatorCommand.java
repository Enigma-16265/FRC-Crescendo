package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
// import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;

public class ElevatorCommand extends Command
{
    private static final double kLiftSlewRateLimit = 0.1;

    private final Elevator          m_elevator;
    private final Supplier<Double>  m_speedSupplier;
    private final SlewRateLimiter   m_slewRateLimiter = new SlewRateLimiter( kLiftSlewRateLimit );

    //Constants
    public final int kSpeedAccelerationLimit = 1;

    public ElevatorCommand(
        Elevator         elevator,
        Supplier<Double> speedSupplier )
    {
        this.m_elevator   = elevator;
        this.m_speedSupplier = speedSupplier;

        addRequirements( elevator );
    }

    @Override
    public void initialize() {}
    
    @Override
    public void execute()
    {

        double requestSpeed = m_speedSupplier.get();
        if ( requestSpeed == 0.0 )
        {
            m_slewRateLimiter.reset( 0.0 );
        }

        boolean positiveDirection = false;
        if ( requestSpeed > 0.0 )
        {
            positiveDirection = true;
        }

        double commandSpeed = m_slewRateLimiter.calculate( requestSpeed );
        m_elevator.lift( commandSpeed, positiveDirection );

    }

    @Override
    public void end( boolean interrupted ) {}

    @Override
    public boolean isFinished() {
        return false;
    }

}
