package frc.robot.subsystems;

import java.util.Map;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.logging.DataNetworkTableLog;
import frc.robot.Constants.ModuleConstants;

// 4 Motors
// private final CANSparkMax m_shooterFlywheelOneSparkMax;
// private final CANSparkMax m_shooterFlywheelTwoSparkMax;
// private final CANSparkMax m_shooterPivotSparkMax;
// private final CANSparkMax m_shooterFeederSparkMax;

// private final RelativeEncoder m_shooterFlywheelOneEncoder;
// private final RelativeEncoder m_shooterFlywheelTwoEncoder;
// private final RelativeEncoder m_shooterPivotEncoder;
// private final RelativeEncoder m_shooterFeederEncoder;

// private final SparkPIDController m_shooterFlywheelOnePIDController;
// private final SparkPIDController m_shooterFlywheelPIDController;
// private final SparkPIDController m_shooterPivotPIDController;
// private final SparkPIDController m_shooterFeederPIDController;

public class Shooter extends SubsystemBase
{
    private static final DataNetworkTableLog dataLog =
    new DataNetworkTableLog( 
        "Subsystems.Shooter",
        Map.of( "speed", DataNetworkTableLog.COLUMN_TYPE.DOUBLE,
                "holdPosition", DataNetworkTableLog.COLUMN_TYPE.DOUBLE ) );

    // Can IDs
    public static final int kShooterFlywheelRightCanID = 16;
    public static final int kShooterFlywheelLeftCanID = 17;

    private static final double ANGLE_TOLERANCE = 2.0; // degrees
    private static final double maxAngle = 90.0; // maximum angle
    private static final double minAngle = 0.0; // minimum angle
    
    private static final double kP = 0.1;
    private static final double kI = 0.0;
    private static final double kD = 0.0;

    // Two fly-wheel motors
    private final CANSparkMax        m_shooterFlywheelRightSparkMax;
    private final CANSparkMax        m_shooterFlywheelLeftSparkMax;
    private final RelativeEncoder    m_shooterFlywheelEncoder;
    private final SparkPIDController m_shooterFlywheelPIDController;

    private double holdPosition = -1.0;

    public Shooter()
    {

        // Flywheel
        m_shooterFlywheelRightSparkMax = new CANSparkMax(kShooterFlywheelRightCanID, MotorType.kBrushless);
        m_shooterFlywheelLeftSparkMax = new CANSparkMax(kShooterFlywheelLeftCanID, MotorType.kBrushless);

        m_shooterFlywheelLeftSparkMax.follow(m_shooterFlywheelRightSparkMax, true);

        m_shooterFlywheelEncoder = m_shooterFlywheelRightSparkMax.getEncoder();
        m_shooterFlywheelEncoder.setPositionConversionFactor( 1.0 / 36.0 );
        m_shooterFlywheelEncoder.setPosition( 0.0 );
        
        m_shooterFlywheelPIDController = m_shooterFlywheelRightSparkMax.getPIDController();

        m_shooterFlywheelPIDController.setFeedbackDevice( m_shooterFlywheelEncoder );

        m_shooterFlywheelPIDController.setP(kP);
        m_shooterFlywheelPIDController.setI(kI);
        m_shooterFlywheelPIDController.setD(kD);

        m_shooterFlywheelPIDController.setOutputRange(-1.0, 1.0);

    }

    public void spin( double speed )
    {
        dataLog.publish( "speed", speed );

        if ( speed != 0.0 )
        {
            
            m_shooterFlywheelRightSparkMax.set( speed );

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
                holdPosition = m_shooterFlywheelEncoder.getPosition();
                dataLog.publish( "holdPosition", holdPosition );
            }

            m_shooterFlywheelPIDController.setReference( holdPosition, CANSparkMax.ControlType.kPosition );
        }

    }

}
