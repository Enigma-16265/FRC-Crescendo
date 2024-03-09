package frc.robot.subsystems;

import java.util.Map;

import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.logging.DataNetworkTableLog;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

public class Elevator extends SubsystemBase
{

    private static final DataNetworkTableLog dataLog =
    new DataNetworkTableLog( 
        "Subsystems.ElevatorLift",
        Map.of( "speed", DataNetworkTableLog.COLUMN_TYPE.DOUBLE,
                "controlMode", DataNetworkTableLog.COLUMN_TYPE.STRING,
                "setPointPos", DataNetworkTableLog.COLUMN_TYPE.DOUBLE,
                "inputMode", DataNetworkTableLog.COLUMN_TYPE.STRING ) );

    // Can ID
    public static final int kElevatorRightCanID = 12;
    public static final int kElevatorLeftCanID = 13;

    // PID
    private static final double kP = 0.1;
    private static final double kI = 0.0;
    private static final double kD = 0.0;

    // Constants
    public static final double kPositionConversionFactor = 1.0/25.0;
    public static final double kPullyDiamaterM = 38.82/1000;

    // Switch Channel
    public static final int kLimitSwitchChannel = 0;

    // Modes
    private ControlMode m_controlMode = ControlMode.UNSET;
    private InputMode   m_inputMode   = InputMode.NOMINAL;
    
    // Two Motors
    private final CANSparkMax m_elevatorRightSparkMax;
    private final CANSparkMax m_elevatorLeftSparkMax;

    private final RelativeEncoder m_elevatorLiftEncoder;

    private final SparkPIDController m_elevatorPIDController;

    private double m_setPointPos = 0.0;

    // Limit
    DigitalInput m_limitSwitch = new DigitalInput( kLimitSwitchChannel );

    public Elevator()
    {
        
        m_elevatorRightSparkMax = new CANSparkMax(kElevatorRightCanID, MotorType.kBrushless);
        m_elevatorLeftSparkMax = new CANSparkMax(kElevatorLeftCanID, MotorType.kBrushless);

        m_elevatorLeftSparkMax.follow(m_elevatorRightSparkMax, true);

        m_elevatorLiftEncoder = m_elevatorRightSparkMax.getEncoder();
        m_elevatorLiftEncoder.setPositionConversionFactor(kPositionConversionFactor);
        m_elevatorLiftEncoder.setPosition( 0.0 );
        
        m_elevatorPIDController = m_elevatorRightSparkMax.getPIDController();

        m_elevatorPIDController.setFeedbackDevice( m_elevatorLiftEncoder );

        m_elevatorPIDController.setP(kP);
        m_elevatorPIDController.setI(kI);
        m_elevatorPIDController.setD(kD);

        m_elevatorPIDController.setOutputRange(-1.0, 1.0);

    }

    public InputMode getInputMode()
    {
        return m_inputMode;
    }

    public void lift( double speed, boolean positiveDirection )
    {
        dataLog.publish( "speed", speed );

        if ( ( speed != 0.0 ) && m_limitSwitch.get() )
        {

            if ( m_elevatorLiftEncoder.getPosition() <=  2.0 )
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

            m_elevatorRightSparkMax.set( speed );

        }
        else
        {

            if ( m_controlMode != ControlMode.HOLD )
            {
                m_controlMode = ControlMode.HOLD;
                m_setPointPos = m_elevatorLiftEncoder.getPosition();

                dataLog.publish( "controlMode", m_controlMode );
                dataLog.publish( "setPointPos", m_setPointPos );
            }

            m_elevatorPIDController.setReference( m_setPointPos, CANSparkMax.ControlType.kPosition );
        }

    }

    public void home( double speed )
    {
        double driveDownSpeed = -1.0 * Math.abs( speed );

        if ( !m_limitSwitch.get() )
        {
            m_elevatorRightSparkMax.set( driveDownSpeed );
        }
        else
        {
            m_elevatorLiftEncoder.setPosition( 0.0 );
            m_inputMode = InputMode.LOWER_LIMIT;
        }

    }

}
