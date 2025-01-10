package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;

import java.util.Map;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.logging.DataNetworkTableLog;

public class Intake extends SubsystemBase
{
    private static final DataNetworkTableLog dataLog =
        new DataNetworkTableLog( 
            "Subsystems.Intake",
            Map.of( "speed", DataNetworkTableLog.COLUMN_TYPE.DOUBLE,
                    "holdPosition", DataNetworkTableLog.COLUMN_TYPE.DOUBLE ) );

    // Can IDs
    public static final int kIntakeWheelCanID = 15;

    // PID
    private static final double kP = 0.0;
    private static final double kI = 0.0;
    private static final double kD = 0.0;

    // Motor
    private final SparkMax m_intakeWheelSparkMax;

    private final RelativeEncoder m_intakeWheelEncoder;

    private final SparkClosedLoopController m_intakePIDController;

    private double holdPosition = -1.0;

    public final SparkMaxConfig m_intakeWheelConfig;

    public Intake()
    {

        m_intakeWheelConfig = new SparkMaxConfig();
        m_intakeWheelConfig.encoder.positionConversionFactor( 1.0/3.0 );
        m_intakeWheelConfig.closedLoop.pid(kP, kI, kD);
        m_intakeWheelConfig.closedLoop.outputRange(-1.0 , 1.0);
        

        // Wheel
        m_intakeWheelSparkMax = new SparkMax(kIntakeWheelCanID, MotorType.kBrushless);
        m_intakeWheelEncoder = m_intakeWheelSparkMax.getEncoder();

        m_intakeWheelEncoder.setPosition( 0.0 );

        m_intakePIDController = m_intakeWheelSparkMax.getClosedLoopController();

    }

    public void roll( double speed )
    {
        // dataLog.publish( "speed", speed );

        if ( speed != 0.0 )
        {
            
            m_intakePIDController.setReference( speed, SparkMax.ControlType.kDutyCycle );

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
                holdPosition = m_intakeWheelEncoder.getPosition();
                // dataLog.publish( "holdPosition", holdPosition );
            }

            m_intakePIDController.setReference( holdPosition, SparkMax.ControlType.kPosition );
        }

    }

}
