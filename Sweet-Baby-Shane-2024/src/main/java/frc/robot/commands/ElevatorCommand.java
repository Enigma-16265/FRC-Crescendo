package frc.robot.commands;

import java.util.Map;
import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
// import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import frc.logging.DataNetworkTableLog;
import frc.robot.subsystems.Elevator;

public class ElevatorCommand extends Command
{

    private static final DataNetworkTableLog dataLog =
    new DataNetworkTableLog( 
        "Subsystems.ElevatorLift.DefaultCommand",
        Map.of( "requestSpeed", DataNetworkTableLog.COLUMN_TYPE.DOUBLE,
                "commandSpeed", DataNetworkTableLog.COLUMN_TYPE.DOUBLE ) );

    private static final double kLiftSlewRateLimit = 0.5;
    // private static final double kLowerSlewRateLimit = 0.5;

    private final Elevator          m_elevator;
    private final Supplier<Double>  m_speedSupplier;
    private final SlewRateLimiter   m_liftSlewRateLimiter = new SlewRateLimiter( kLiftSlewRateLimit );
    // private final SlewRateLimiter   m_lowerSlewRateLimiter = new SlewRateLimiter( kLowerSlewRateLimit );


    //Constants
    public final double kSpeedAccelerationLimit = 0.8;

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
            m_liftSlewRateLimiter.reset( 0.0 );
        }

        boolean positiveDirection = false;
        if ( requestSpeed > 0.0 )
        {
            positiveDirection = true;
        }

        double commandSpeed = m_liftSlewRateLimiter.calculate( requestSpeed );

        // dataLog.publish( "requestSpeed", requestSpeed );
        // dataLog.publish( "commandSpeed", commandSpeed );

        m_elevator.lift( commandSpeed, positiveDirection );

    }

    @Override
    public void end( boolean interrupted ) {}

    @Override
    public boolean isFinished() {
        return false;
    }

}
