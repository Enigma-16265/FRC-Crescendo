package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.InputMode;
import frc.robot.subsystems.IntakePivot;

public class IntakePivotLimitCommand  extends Command
{
    public enum Behavior
    {
        STOW,
        GO
    }

    //Constants
    private static final double kDriveSpeed = 0.8;

    private final IntakePivot m_intakePivot;
    private final Behavior    m_behavior;
    private final double      m_driveSpeed;
    private final boolean     m_positiveDir;
    private final InputMode   m_exitCondition;

    public IntakePivotLimitCommand( IntakePivot intakePivot,
                                    Behavior    behavior )
    {
        m_intakePivot = intakePivot;
        m_behavior = behavior;

        switch ( m_behavior )
        {
            case STOW:
                m_driveSpeed = kDriveSpeed;
                break;
            case GO:
                m_driveSpeed = -kDriveSpeed;
                break;
            default:
                // Do nothing
                m_driveSpeed = 0.0;
                break;
        }

        if ( m_driveSpeed > 0.0 )
        {
            m_positiveDir   = true;
            m_exitCondition = InputMode.UPPER_LIMIT;
        }
        else if ( m_driveSpeed < 0.0 )
        {
            m_positiveDir   = false;
            m_exitCondition = InputMode.LOWER_LIMIT;
        }
        else
        {
            m_positiveDir   = true;
            m_exitCondition = InputMode.NOMINAL;
        }

        addRequirements( m_intakePivot );
    }

    @Override
    public void initialize() {}
    
    @Override
    public void execute()
    {
        m_intakePivot.slew( m_driveSpeed, m_positiveDir );
    }

    @Override
    public void end( boolean interrupted ) {}

    @Override
    public boolean isFinished()
    {
        return ( m_intakePivot.getInputMode() == m_exitCondition );
    }

}