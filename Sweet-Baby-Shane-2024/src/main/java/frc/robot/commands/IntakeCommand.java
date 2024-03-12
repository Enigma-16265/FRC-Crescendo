package frc.robot.commands;

import java.util.Map;
import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import frc.logging.DataNetworkTableLog;
import frc.robot.subsystems.Intake;

public class IntakeCommand extends Command 
{

    private static final double kLiftSlewRateLimit = 0.3;

    private final Intake m_intake;
    private final Supplier<Double> m_speedSupplier;
    private final SlewRateLimiter  m_slewRateLimiter = new SlewRateLimiter( kLiftSlewRateLimit );

    public IntakeCommand(
        Intake intake,
        Supplier<Double> speedSupplier )
    {
        this.m_intake = intake;
        this.m_speedSupplier = speedSupplier;

        addRequirements(intake);
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

        double commandSpeed = m_slewRateLimiter.calculate( requestSpeed );

        m_intake.roll( commandSpeed );

    }

    @Override
    public void end( boolean interrupted ) {}

    @Override
    public boolean isFinished() {
        return false;
    }

}
