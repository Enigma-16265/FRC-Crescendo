package frc.robot.subsystems;

import java.util.Map;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.logging.DataNetworkTableLog;

public class Shooter extends SubsystemBase
{
    private static final DataNetworkTableLog dataLog =
    new DataNetworkTableLog( 
        "Subsystems.Shooter",
        Map.of( "speed", DataNetworkTableLog.COLUMN_TYPE.DOUBLE,
                "holdPosition", DataNetworkTableLog.COLUMN_TYPE.DOUBLE ) );

    // Can IDs
    public static final int kShooterFlywheelRightCanID = 14;
    public static final int kShooterFlywheelLeftCanID = 13;
    
    private static final double kP = 0.0;
    private static final double kI = 0.0;
    private static final double kD = 0.0;

    // Two fly-wheel motors
    private final CANSparkFlex m_shooterFlywheelRightSparkFlex;
    private final CANSparkFlex m_shooterFlywheelLeftSparkFlex;

    private final RelativeEncoder m_shooterFlywheelEncoder;

    private final SparkPIDController m_shooterFlywheelPIDController;

    private double holdPosition = -1.0;

    public Shooter()
    {

        // Flywheel
        m_shooterFlywheelRightSparkFlex = new CANSparkFlex(kShooterFlywheelRightCanID, MotorType.kBrushless);
        m_shooterFlywheelLeftSparkFlex = new CANSparkFlex(kShooterFlywheelLeftCanID, MotorType.kBrushless);
        m_shooterFlywheelRightSparkFlex.setIdleMode( IdleMode.kBrake );
        m_shooterFlywheelLeftSparkFlex.setIdleMode( IdleMode.kBrake );

        m_shooterFlywheelLeftSparkFlex.follow(m_shooterFlywheelRightSparkFlex, true);

        m_shooterFlywheelEncoder = m_shooterFlywheelRightSparkFlex.getEncoder();
        m_shooterFlywheelEncoder.setPositionConversionFactor( 1.0 );
        m_shooterFlywheelEncoder.setPosition( 0.0 );
        
        m_shooterFlywheelPIDController = m_shooterFlywheelRightSparkFlex.getPIDController();

        m_shooterFlywheelPIDController.setFeedbackDevice( m_shooterFlywheelEncoder );

        m_shooterFlywheelPIDController.setP(kP);
        m_shooterFlywheelPIDController.setI(kI);
        m_shooterFlywheelPIDController.setD(kD);

        m_shooterFlywheelPIDController.setOutputRange(-1.0, 1.0);

    }

    public void spin( double speed )
    {
        // dataLog.publish( "speed", speed );

        if ( speed != 0.0 )
        {
            
            m_shooterFlywheelPIDController.setReference( speed, CANSparkMax.ControlType.kDutyCycle );

            if ( holdPosition >= 0.0 )
            {
                holdPosition = -1.0;
                // dataLog.publish( "holdPosition", holdPosition );
            }
        }
        else
        {

            if ( holdPosition < 0.0 )
            {
                holdPosition = m_shooterFlywheelEncoder.getPosition();
                // dataLog.publish( "holdPosition", holdPosition );
            }

            m_shooterFlywheelPIDController.setReference( holdPosition, CANSparkMax.ControlType.kPosition );
        }

    }

    public void stop(){

        m_shooterFlywheelLeftSparkFlex.set( 0.0 );
        m_shooterFlywheelRightSparkFlex.set( 0.0 );

    }

}
