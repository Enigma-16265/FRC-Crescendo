package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class IntakeSlowReverseCommand extends Command
{
    private final Intake m_intake;

    //Constants
    public final int kSpeedAccelerationLimit = 1;

    public IntakeSlowReverseCommand(
        Intake intake )
    {
        m_intake = intake;

        addRequirements( intake );
    }

    @Override
    public void initialize() {}
    
    @Override
    public void execute()
    {

        m_intake.roll( 0.2 );

    }

    @Override
    public void end( boolean interrupted ) {}

    @Override
    public boolean isFinished() {
        return false;
    }
}
