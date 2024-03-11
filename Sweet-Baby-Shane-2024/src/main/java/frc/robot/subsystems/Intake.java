package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import java.util.Map;

import com.revrobotics.CANSparkLowLevel.MotorType;

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
    private final CANSparkMax m_intakeWheelSparkMax;

    private final RelativeEncoder m_intakeWheelEncoder;

    private final SparkPIDController m_intakePIDController;

    private double holdPosition = -1.0;

    public Intake()
    {

        // Wheel
        m_intakeWheelSparkMax = new CANSparkMax(kIntakeWheelCanID, MotorType.kBrushless);

        m_intakeWheelEncoder = m_intakeWheelSparkMax.getEncoder();
        m_intakeWheelEncoder.setPositionConversionFactor( 1.0 / 3.0 );
        m_intakeWheelEncoder.setPosition( 0.0 );
        
        m_intakePIDController = m_intakeWheelSparkMax.getPIDController();

        m_intakePIDController.setFeedbackDevice( m_intakeWheelEncoder );

        m_intakePIDController.setP(kP);
        m_intakePIDController.setI(kI);
        m_intakePIDController.setD(kD);

        m_intakePIDController.setOutputRange(-1.0, 1.0);

    }

    public void roll( double speed )
    {
        dataLog.publish( "speed", speed );

        if ( speed != 0.0 )
        {
            
            m_intakePIDController.setReference( speed, CANSparkMax.ControlType.kDutyCycle );

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
                holdPosition = m_intakeWheelEncoder.getPosition();
                dataLog.publish( "holdPosition", holdPosition );
            }

            m_intakePIDController.setReference( holdPosition, CANSparkMax.ControlType.kPosition );
        }

    }

}
