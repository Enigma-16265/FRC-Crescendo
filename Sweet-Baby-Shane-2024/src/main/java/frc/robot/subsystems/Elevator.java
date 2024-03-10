package frc.robot.subsystems;

import java.util.Map;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotBase;
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
                "posDir", DataNetworkTableLog.COLUMN_TYPE.STRING,
                "controlMode", DataNetworkTableLog.COLUMN_TYPE.STRING,
                "inputMode", DataNetworkTableLog.COLUMN_TYPE.STRING,
                "encoderPos", DataNetworkTableLog.COLUMN_TYPE.DOUBLE,
                "limitSwitch", DataNetworkTableLog.COLUMN_TYPE.STRING,
                "setPointPos", DataNetworkTableLog.COLUMN_TYPE.DOUBLE,
                "simEncoderPos", DataNetworkTableLog.COLUMN_TYPE.DOUBLE,
                "homeMode", DataNetworkTableLog.COLUMN_TYPE.STRING ) );

    // Can ID
    public static final int kElevatorRightCanID = 10;
    public static final int kElevatorLeftCanID = 9;

    // PID
    private static final double kP = 0.0;
    private static final double kI = 0.0;
    private static final double kD = 0.0;

    // Constants
    public static final double kEncoderCloseToZero = 2.0;
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
    private final RelativeEncoder m_elevatorLeftEncoder;

    private final SparkPIDController m_elevatorPIDController;

    private double m_setPointPos = 0.0;

    // Limit
    DigitalInput m_limitSwitch = new DigitalInput( kLimitSwitchChannel );

    public Elevator()
    {
        
        m_elevatorRightSparkMax = new CANSparkMax(kElevatorRightCanID, MotorType.kBrushless);
        m_elevatorLeftSparkMax = new CANSparkMax(kElevatorLeftCanID, MotorType.kBrushless);

        m_elevatorRightSparkMax.setIdleMode( IdleMode.kBrake );
        m_elevatorLeftSparkMax.setIdleMode( IdleMode.kBrake );

        m_elevatorLeftSparkMax.follow(m_elevatorRightSparkMax );

        m_elevatorLiftEncoder = m_elevatorRightSparkMax.getEncoder();
        m_elevatorLiftEncoder.setPositionConversionFactor(kPositionConversionFactor);
        m_elevatorLiftEncoder.setPosition( 0.0 );

        m_elevatorLeftEncoder = m_elevatorLeftSparkMax.getEncoder();
        m_elevatorLeftEncoder.setPositionConversionFactor(kPositionConversionFactor);
        m_elevatorLeftEncoder.setPosition( 0.0 );
        
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
        dataLog.publish( "posDir", positiveDirection );

        double  encoderPos        = m_elevatorLiftEncoder.getPosition();
        boolean limitSwitchActive = !m_limitSwitch.get();

        if ( RobotBase.isSimulation() )
        {
            encoderPos        = getSimEncoderPos();
            limitSwitchActive = !getSimLimitSwitch();
        }

        dataLog.publish( "encoderPos", encoderPos );
        dataLog.publish( "limitSwitch", limitSwitchActive );

        if ( ( speed != 0.0 ) && limitSwitchActive )
        {

            if ( encoderPos <= kEncoderCloseToZero )
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

            m_elevatorPIDController.setReference( speed, CANSparkMax.ControlType.kDutyCycle );

            if ( RobotBase.isSimulation() )
            {
                updateSimEncoder( speed );
            }

        }
        else
        {

            if ( m_controlMode != ControlMode.HOLD )
            {
                m_controlMode = ControlMode.HOLD;
                m_setPointPos = encoderPos;

                dataLog.publish( "controlMode", m_controlMode );
                dataLog.publish( "setPointPos", m_setPointPos );
            }

            m_elevatorPIDController.setReference( 0.0, CANSparkMax.ControlType.kDutyCycle );
        }

    }

    private HomeMode m_homeMode = HomeMode.LOST;

    public void home( double speed )
    {
        
        double driveDownSpeed = -1.0 * Math.abs( speed );

        boolean limitSwitchActive = !m_limitSwitch.get();
        if ( RobotBase.isSimulation() )
        {
            limitSwitchActive = !getSimLimitSwitch();
        }


        if ( !limitSwitchActive )
        {
            m_homeMode = HomeMode.DRIVE_DOWN;
            m_elevatorPIDController.setReference( driveDownSpeed, CANSparkMax.ControlType.kDutyCycle );

            if ( RobotBase.isSimulation() )
            {
                updateSimEncoder( speed );
            }
        }
        else
        {
            m_homeMode = HomeMode.HOMED;
            m_inputMode = InputMode.LOWER_LIMIT;
            m_elevatorLiftEncoder.setPosition( 0.0 );
            m_elevatorPIDController.setReference( 0.0, CANSparkMax.ControlType.kPosition );
        }

        dataLog.publish( "homeMode", m_homeMode );

        double  encoderPos = m_elevatorLiftEncoder.getPosition();
        if ( RobotBase.isSimulation() )
        {
            encoderPos = getSimEncoderPos();
        }

        dataLog.publish( "encoderPos", encoderPos );

    }

    // Sim methods
    private double m_simEncoderPos = 0.0;

    private void updateSimEncoder( double speed )
    {
        m_simEncoderPos += speed;
        dataLog.publish( "simEncoderPos", m_simEncoderPos );
    }

    private boolean getSimLimitSwitch()
    {

        boolean limitSwitchPulledHigh = true;

        if ( m_simEncoderPos < -5.0 ) 
        {
            limitSwitchPulledHigh = false;
        }
        else if ( m_simEncoderPos > 5.0 )
        {
            limitSwitchPulledHigh = false;
        }

        return limitSwitchPulledHigh;
    }

    private double getSimEncoderPos()
    {
        return m_simEncoderPos;
    }


}
