package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;

public class ElevatorHomingCommand extends Command
{

    private static final double kDriveDownSpeed = -0.1;

    private final Elevator m_elevator;

    //Constants
    public final int kSpeedAccelerationLimit = 1;

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
        System.out.println( "!!!!!!!!!!!!!!!! DID THIS RUN! !!!!!!!!!!!!!!!" );
        m_elevator.home( kDriveDownSpeed );
    }

    @Override
    public void end( boolean interrupted ) {}

    @Override
    public boolean isFinished() {
        return ( m_elevator.getInputMode() == Elevator.InputMode.LOWER_LIMIT );
    }

}
