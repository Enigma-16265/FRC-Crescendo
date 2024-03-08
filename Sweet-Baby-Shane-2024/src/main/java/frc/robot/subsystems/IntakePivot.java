package frc.robot.subsystems;

import java.util.Map;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.logging.DataNetworkTableLog;

public class IntakePivot extends SubsystemBase
{
    private static final DataNetworkTableLog dataLog =
        new DataNetworkTableLog( 
            "Subsystems.IntakePivot",
            Map.of( "desiredSpeed", DataNetworkTableLog.COLUMN_TYPE.DOUBLE,
                    "commandedSpeed", DataNetworkTableLog.COLUMN_TYPE.DOUBLE,
                    "holdPosition", DataNetworkTableLog.COLUMN_TYPE.DOUBLE ) );

    // Can IDs
    public static final int kIntakePivotCanID = 15;

    private static final double SHOOTER_SLEW_RATE_LIMIT = 0.1;

    private static final double kP = 0.1;
    private static final double kI = 0.0;
    private static final double kD = 0.0;

    // 1 Motors
    private final CANSparkMax        m_intakePivotSparkMax;
    private final RelativeEncoder    m_intakePivotEncoder;
    private final SparkPIDController m_intakePIDController;

    private final SlewRateLimiter    m_slewRateLimiter = new SlewRateLimiter( SHOOTER_SLEW_RATE_LIMIT );

    private double holdPosition = -1.0;

    public IntakePivot()
    {
        // Pivot
        m_intakePivotSparkMax = new CANSparkMax(kIntakePivotCanID, MotorType.kBrushless);

        m_intakePivotEncoder = m_intakePivotSparkMax.getEncoder();
        m_intakePivotEncoder.setPositionConversionFactor( 1.0 / 50.0 );
        m_intakePivotEncoder.setPosition( 0.0 );
        
        m_intakePIDController = m_intakePivotSparkMax.getPIDController();

        m_intakePIDController.setFeedbackDevice( m_intakePivotEncoder );

        m_intakePIDController.setP(kP);
        m_intakePIDController.setI(kI);
        m_intakePIDController.setD(kD);

        m_intakePIDController.setOutputRange(-1.0, 1.0);
    }

    public void slew( double desiredSpeed )
    {
        dataLog.publish( "desiredSpeed", desiredSpeed );

        if ( desiredSpeed != 0.0 )
        {
            double commandedSpeed = m_slewRateLimiter.calculate( desiredSpeed );
            
            dataLog.publish( "commandedSpeed", commandedSpeed );

            m_intakePivotSparkMax.set( commandedSpeed );

            if ( holdPosition >= 0.0 )
            {
                holdPosition = -1.0;
                dataLog.publish( "holdPosition", holdPosition );
            }
        }
        else
        {

            if ( holdPosition < 0.0 )
            {
                holdPosition = m_intakePivotEncoder.getPosition();
                dataLog.publish( "holdPosition", holdPosition );
            }

            m_intakePIDController.setReference( holdPosition, CANSparkMax.ControlType.kPosition );
        }

    }

}
