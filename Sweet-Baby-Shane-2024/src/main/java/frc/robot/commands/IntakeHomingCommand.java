package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakePivot;

public class IntakeHomingCommand extends Command
{

    private static final double kDriveDownSpeed = -0.1;

    private final IntakePivot m_intake;

    public IntakeHomingCommand( IntakePivot intake )
    {
        m_intake = intake;
        addRequirements( m_intake );
    }

    @Override
    public void initialize() {}
    
    @Override
    public void execute()
    {
        System.out.println( "IntakeHomingCommand execute!!!!!!!!!!!!!!!" );
        m_intake.home( kDriveDownSpeed );
    }

    @Override
    public void end( boolean interrupted ) {}

    @Override
    public boolean isFinished() {
        return ( m_intake.getInputMode() == IntakePivot.InputMode.LOWER_LIMIT );
    }

}
