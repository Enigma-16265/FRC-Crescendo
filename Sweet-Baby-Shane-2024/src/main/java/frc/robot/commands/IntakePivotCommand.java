package frc.robot.commands;

import java.util.Map;
import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import frc.logging.DataNetworkTableLog;
import frc.robot.subsystems.IntakePivot;

public class IntakePivotCommand extends Command
{

    private static final DataNetworkTableLog dataLog =
        new DataNetworkTableLog( 
            "Subsystems.IntakePivot.DefaultCommand",
            Map.of( "requestSpeed", DataNetworkTableLog.COLUMN_TYPE.DOUBLE,
                    "commandSpeed", DataNetworkTableLog.COLUMN_TYPE.DOUBLE ) );

    private static final double kLiftSlewRateLimit = 0.1;

    private final IntakePivot      m_intakePivot;
    private final Supplier<Double> m_speedSupplier;
    private final SlewRateLimiter  m_slewRateLimiter = new SlewRateLimiter( kLiftSlewRateLimit );

    //Constants
    public final int kSpeedAccelerationLimit = 1;

    public IntakePivotCommand(
        IntakePivot      intakePivot,
        Supplier<Double> speedSupplier )
    {
        this.m_intakePivot   = intakePivot;
        this.m_speedSupplier = speedSupplier;

        addRequirements( intakePivot );
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

        dataLog.publish( "requestSpeed", requestSpeed );
        dataLog.publish( "commandSpeed", commandSpeed );

        m_intakePivot.slew( commandSpeed, positiveDirection );

    }

    @Override
    public void end( boolean interrupted ) {}

    @Override
    public boolean isFinished() {
        return false;
    }

}