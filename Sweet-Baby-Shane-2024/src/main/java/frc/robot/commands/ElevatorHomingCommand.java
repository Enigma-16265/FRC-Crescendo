package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.InputMode;

public class ElevatorHomingCommand extends Command
{

    //Constants
    private static final double kDriveDownSpeed = -0.1;

    private final Elevator m_elevator;

    public ElevatorHomingCommand( Elevator elevator )
    {
        m_elevator = elevator;
        addRequirements( m_elevator );
    }

    @Override
    public void initialize() {}
    
    @Override
    public void execute()
    {
        m_elevator.home( kDriveDownSpeed );
    }

    @Override
    public void end( boolean interrupted ) {}

    @Override
    public boolean isFinished() {
        return ( m_elevator.getInputMode() == InputMode.LOWER_LIMIT );
    }

}
