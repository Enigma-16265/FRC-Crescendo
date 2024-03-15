package frc.robot.subsystems;

import java.util.Map;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.logging.DataNetworkTableLog;

public class IntakePivot extends SubsystemBase
{
    private static final DataNetworkTableLog dataLog =
        new DataNetworkTableLog( 
            "Subsystems.IntakePivot",
            Map.of( "speed", DataNetworkTableLog.COLUMN_TYPE.DOUBLE,
                    "posDir", DataNetworkTableLog.COLUMN_TYPE.STRING,
                    "controlMode", DataNetworkTableLog.COLUMN_TYPE.STRING,
                    "inputMode", DataNetworkTableLog.COLUMN_TYPE.STRING,
                    "encoderPos", DataNetworkTableLog.COLUMN_TYPE.DOUBLE,
                    "limitSwitch", DataNetworkTableLog.COLUMN_TYPE.STRING,
                    "setPointPos", DataNetworkTableLog.COLUMN_TYPE.DOUBLE,
                    "simEncoderPos", DataNetworkTableLog.COLUMN_TYPE.DOUBLE,
                    "homeMode", DataNetworkTableLog.COLUMN_TYPE.STRING ) );

    // Can IDs
    public static final int kIntakePivotCanID = 16;

    // PID
    private static final double kP = 0.0;
    private static final double kI = 0.0;
    private static final double kD = 0.0;

    // Constants
    public static final double kEncoderResolution = 42.0;
    public static final double kGearRatio         = 25.0;
    public static final double kCountsPerRev      = kEncoderResolution*kGearRatio;

    public static final double kEncoderCloseToZero = 10.0;

    public static final double kEncoderRevUpperLimit = 55.0;
    public static final double kEncoderRevLowerLimit = 0.0;

    // Switch Channel
    public static final int kLimitSwitchChannel = 1;

    private ControlMode m_controlMode = ControlMode.UNSET;
    private InputMode   m_inputMode   = InputMode.NOMINAL;

    // Motor
    private final CANSparkMax m_intakePivotSparkMax;

    private final RelativeEncoder m_intakePivotEncoder;
    
    private final SparkPIDController m_intakePIDController;

    // Limit Switch
    DigitalInput m_limitSwitch = new DigitalInput( kLimitSwitchChannel );

    private double m_setPointPos = 0.0;

    public IntakePivot()
    {
        // Pivot
        m_intakePivotSparkMax = new CANSparkMax(kIntakePivotCanID, MotorType.kBrushless);
        m_intakePivotSparkMax.setIdleMode( IdleMode.kBrake );
        m_intakePivotSparkMax.setInverted( true );

        m_intakePivotEncoder = m_intakePivotSparkMax.getEncoder();
        m_intakePivotEncoder.setPositionConversionFactor( kCountsPerRev );
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
        // dataLog.publish( "speed", speed );
        // dataLog.publish( "posDir", positiveDirection );

        double  encoderPos        = m_intakePivotEncoder.getPosition() / kCountsPerRev;
        boolean limitSwitchActive = (!m_limitSwitch.get() || (encoderPos >= kEncoderRevUpperLimit) || (encoderPos <= kEncoderRevLowerLimit));

        if ( RobotBase.isSimulation() )
        {
            encoderPos        = getSimEncoderPos();
            limitSwitchActive = !getSimLimitSwitch();
        }

        // dataLog.publish( "encoderPos", encoderPos );
        // dataLog.publish( "limitSwitch", limitSwitchActive );

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

        // dataLog.publish( "inputMode", m_inputMode );
        
        if ( speed != 0.0 )
        {

            if ( m_controlMode != ControlMode.ACTIVE )
            {
                m_controlMode = ControlMode.ACTIVE;
                m_setPointPos = 0.0;

                // dataLog.publish( "controlMode", m_controlMode );
                // dataLog.publish( "setPointPos", m_setPointPos );
            }

            m_intakePIDController.setReference( speed, CANSparkMax.ControlType.kDutyCycle );

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

                // dataLog.publish( "controlMode", m_controlMode );
                // dataLog.publish( "setPointPos", m_setPointPos );
            }

            // For now we will use a zero duty cycle to stop the motor, until we can figure out the
            // PIDController position hold
            m_intakePIDController.setReference( 0.0, CANSparkMax.ControlType.kDutyCycle );
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

            m_intakePIDController.setReference( driveDownSpeed, CANSparkMax.ControlType.kDutyCycle );

            if ( RobotBase.isSimulation() )
            {
                updateSimEncoder( speed );
            }
        }
        else
        {
            m_homeMode = HomeMode.HOMED;
            m_inputMode = InputMode.LOWER_LIMIT;
            m_intakePivotEncoder.setPosition( 0.0 );
            m_intakePIDController.setReference( 0.0, CANSparkMax.ControlType.kPosition );
        }

        // dataLog.publish( "homeMode", m_homeMode );

        double  encoderPos = m_intakePivotEncoder.getPosition() / kCountsPerRev;
        if ( RobotBase.isSimulation() )
        {
            encoderPos = getSimEncoderPos();
        }

        // dataLog.publish( "encoderPos", encoderPos );

    }

    // Sim methods
    private double m_simEncoderPos = 0.0;

    private void updateSimEncoder( double speed )
    {
        m_simEncoderPos += speed;
        // dataLog.publish( "simEncoderPos", m_simEncoderPos );
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
