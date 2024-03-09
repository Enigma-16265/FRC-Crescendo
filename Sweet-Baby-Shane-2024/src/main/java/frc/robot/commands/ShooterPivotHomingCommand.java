package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.InputMode;
import frc.robot.subsystems.ShooterPivot;

public class ShooterPivotHomingCommand extends Command
{

    //Constants
    private static final double kDriveDownSpeed = -0.1;

    private final ShooterPivot m_shooterPivot;

    public ShooterPivotHomingCommand( ShooterPivot shooterPivot )
    {
        m_shooterPivot = shooterPivot;
        addRequirements( m_shooterPivot );
    }

    @Override
    public void initialize() {}
    
    @Override
    public void execute()
    {
        System.out.println( "ShooterPivot execute!!!!!!!!!!!!!" );
        m_shooterPivot.home( kDriveDownSpeed );
    }

    @Override
    public void end( boolean interrupted ) {}

    @Override
    public boolean isFinished() {
        return ( m_shooterPivot.getInputMode() == InputMode.LOWER_LIMIT );
    }

}