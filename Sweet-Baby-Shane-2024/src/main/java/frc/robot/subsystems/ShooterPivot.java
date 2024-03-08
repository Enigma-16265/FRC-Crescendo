package frc.robot.subsystems;

import java.util.Map;

import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import frc.logging.DataNetworkTableLog;

public class ShooterPivot extends SubsystemBase
{
    
    private static final DataNetworkTableLog dataLog =
        new DataNetworkTableLog( 
            "Subsystems.ShooterPivot",
            Map.of( "desiredSpeed", DataNetworkTableLog.COLUMN_TYPE.DOUBLE,
                    "commandedSpeed", DataNetworkTableLog.COLUMN_TYPE.DOUBLE,
                    "holdPosition", DataNetworkTableLog.COLUMN_TYPE.DOUBLE ) );

    // Can IDs
    public static final int kShooterPivotCanID = 18;

    private static final double ANGLE_TOLERANCE = 2.0; // degrees
    private static final double maxAngle = 90.0; // maximum angle
    private static final double minAngle = 0.0; // minimum angle
    private static final double SHOOTER_SLEW_RATE_LIMIT = 0.1;
    
    private static final double kP = 0.1;
    private static final double kI = 0.0;
    private static final double kD = 0.0;

    // One Pivot motor
    private final CANSparkMax        m_shooterPivotSparkMax;
    private final RelativeEncoder    m_shooterPivotEncoder;
    private final SparkPIDController m_shooterPIDController;

    private final SlewRateLimiter    m_slewRateLimiter = new SlewRateLimiter( SHOOTER_SLEW_RATE_LIMIT );

    private double holdPosition = -1.0;

    public ShooterPivot()
    {

        // Pivot
        m_shooterPivotSparkMax = new CANSparkMax(kShooterPivotCanID, MotorType.kBrushless);

        m_shooterPivotEncoder = m_shooterPivotSparkMax.getEncoder();
        m_shooterPivotEncoder.setPositionConversionFactor( 1.0 / 36.0 );
        m_shooterPivotEncoder.setPosition( 0.0 );
        
        m_shooterPIDController = m_shooterPivotSparkMax.getPIDController();

        m_shooterPIDController.setFeedbackDevice( m_shooterPivotEncoder );

        m_shooterPIDController.setP(kP);
        m_shooterPIDController.setI(kI);
        m_shooterPIDController.setD(kD);

        m_shooterPIDController.setOutputRange(-1.0, 1.0);
    }

    public void slew( double desiredSpeed )
    {
        dataLog.publish( "desiredSpeed", desiredSpeed );

        if ( desiredSpeed != 0.0 )
        {
            double commandedSpeed = m_slewRateLimiter.calculate( desiredSpeed );
            
            dataLog.publish( "commandedSpeed", commandedSpeed );

            m_shooterPivotSparkMax.set( commandedSpeed );

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
                holdPosition = m_shooterPivotEncoder.getPosition();
                dataLog.publish( "holdPosition", holdPosition );
            }

            m_shooterPIDController.setReference( holdPosition, CANSparkMax.ControlType.kPosition );
        }

    }

}