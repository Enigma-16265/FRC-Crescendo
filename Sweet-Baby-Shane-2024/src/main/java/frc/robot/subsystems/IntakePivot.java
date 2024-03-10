package frc.robot.subsystems;

import java.util.Map;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.logging.DataNetworkTableLog;

public class IntakePivot extends SubsystemBase
{
    private static final DataNetworkTableLog dataLog =
        new DataNetworkTableLog( 
            "Subsystems.IntakePivot",
            Map.of( "speed", DataNetworkTableLog.COLUMN_TYPE.DOUBLE,
                    "controlMode", DataNetworkTableLog.COLUMN_TYPE.STRING,
                    "setPointPos", DataNetworkTableLog.COLUMN_TYPE.DOUBLE,
                    "inputMode", DataNetworkTableLog.COLUMN_TYPE.STRING ) );

    // Can IDs
    public static final int kIntakePivotCanID = 16;

    private static final double kP = 0.1;
    private static final double kI = 0.0;
    private static final double kD = 0.0;

    // Switch Channel
    public static final int kLimitSwitchChannel = 1;

    private ControlMode m_controlMode = ControlMode.UNSET;
    private InputMode   m_inputMode   = InputMode.NOMINAL;

    // Motor
    private final CANSparkMax        m_intakePivotSparkMax;
    private final RelativeEncoder    m_intakePivotEncoder;
    private final SparkPIDController m_intakePIDController;

    // Limit Switch
    DigitalInput m_limitSwitch = new DigitalInput( kLimitSwitchChannel );

    private double m_setPointPos = 0.0;

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

    public InputMode getInputMode()
    {
        return m_inputMode;
    }

    public void slew( double speed, boolean positiveDirection )
    {
        dataLog.publish( "speed", speed );

        if ( ( speed != 0.0 ) && m_limitSwitch.get() )
        {

            if ( m_intakePivotEncoder.getPosition() <=  2.0 )
            {
                
                m_inputMode = InputMode.LOWER_LIMIT;
                if ( !positiveDirection )
                {
                    speed = 0.0;
                }

            }
            else
            {
                m_inputMode = InputMode.UPPER_LIMIT;
                if ( positiveDirection )
                {
                    speed = 0.0;
                }
            }

        }
        else
        {
            m_inputMode = InputMode.NOMINAL;
        }

        dataLog.publish( "inputMode", m_inputMode );
        
        if ( speed != 0.0 )
        {

            if ( m_controlMode != ControlMode.ACTIVE )
            {
                m_controlMode = ControlMode.ACTIVE;
                m_setPointPos = 0.0;

                dataLog.publish( "controlMode", m_controlMode );
                dataLog.publish( "setPointPos", m_setPointPos );
            }

            m_intakePivotSparkMax.set( speed );

        }
        else
        {

            if ( m_controlMode != ControlMode.HOLD )
            {
                m_controlMode = ControlMode.HOLD;
                m_setPointPos = m_intakePivotEncoder.getPosition();

                dataLog.publish( "controlMode", m_controlMode );
                dataLog.publish( "setPointPos", m_setPointPos );
            }

            m_intakePIDController.setReference( m_setPointPos, CANSparkMax.ControlType.kPosition );
        }

    }

    public void home( double speed )
    {
        double driveDownSpeed = -1.0 * Math.abs( speed );

        if ( !m_limitSwitch.get() )
        {
            m_intakePivotSparkMax.set( driveDownSpeed );
        }
        else
        {
            m_intakePivotEncoder.setPosition( 0.0 );
            m_inputMode = InputMode.LOWER_LIMIT;
        }

    }

}
