package frc.robot.subsystems;

import java.util.Map;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;

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
    private final SparkFlex m_shooterFlywheelRightSparkFlex;
    private final SparkFlex m_shooterFlywheelLeftSparkFlex;

    private final RelativeEncoder m_shooterFlywheelEncoder;

    private final SparkClosedLoopController m_shooterFlywheelPIDController;

    private double holdPosition = -1.0;

    public final SparkFlexConfig m_shooterFlywheelRightSparkFlexConfig;
    public final SparkFlexConfig m_shooterFlywheelLeftSparkFlexConfig;

    public Shooter()
    {

        // Flywheel
        m_shooterFlywheelLeftSparkFlexConfig = new SparkFlexConfig();
        m_shooterFlywheelLeftSparkFlexConfig.idleMode(IdleMode.kBrake);
        m_shooterFlywheelLeftSparkFlexConfig.follow(kShooterFlywheelRightCanID);

        m_shooterFlywheelRightSparkFlexConfig = new SparkFlexConfig();
        m_shooterFlywheelRightSparkFlexConfig.idleMode(IdleMode.kBrake);
        m_shooterFlywheelRightSparkFlexConfig.encoder.positionConversionFactor( 1.0 / 36.0 );
        m_shooterFlywheelRightSparkFlexConfig.closedLoop.pid(kP, kI, kD);
        m_shooterFlywheelRightSparkFlexConfig.closedLoop.outputRange( -1.0, 1.0 );
        
        m_shooterFlywheelRightSparkFlex = new SparkFlex(kShooterFlywheelRightCanID, MotorType.kBrushless);
        m_shooterFlywheelLeftSparkFlex = new SparkFlex(kShooterFlywheelLeftCanID, MotorType.kBrushless);

        m_shooterFlywheelEncoder = m_shooterFlywheelRightSparkFlex.getEncoder();
        m_shooterFlywheelEncoder.setPosition( 0.0 );
        
        m_shooterFlywheelPIDController = m_shooterFlywheelRightSparkFlex.getClosedLoopController();

    }

    public void spin( double speed )
    {
        // dataLog.publish( "speed", speed );

        if ( speed != 0.0 )
        {
            
            m_shooterFlywheelPIDController.setReference( speed, SparkMax.ControlType.kDutyCycle );

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

            m_shooterFlywheelPIDController.setReference( holdPosition, SparkMax.ControlType.kPosition );
        }

    }

}
