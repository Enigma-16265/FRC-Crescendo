package frc.robot.subsystems;

import java.util.Map;

import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotBase;
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
        Map.of( "speed", DataNetworkTableLog.COLUMN_TYPE.DOUBLE,
                "posDir", DataNetworkTableLog.COLUMN_TYPE.STRING,
                "controlMode", DataNetworkTableLog.COLUMN_TYPE.STRING,
                "inputMode", DataNetworkTableLog.COLUMN_TYPE.STRING,
                "encoderPos", DataNetworkTableLog.COLUMN_TYPE.DOUBLE,
                "limitSwitch", DataNetworkTableLog.COLUMN_TYPE.STRING,
                "setPointPos", DataNetworkTableLog.COLUMN_TYPE.DOUBLE,
                "simEncoderPos", DataNetworkTableLog.COLUMN_TYPE.DOUBLE ) );

    // Can IDs
    public static final int kShooterPivotCanID = 12;
    
    // PID
    private static final double kP = 0.1;
    private static final double kI = 0.0;
    private static final double kD = 0.0;

    // Constants
    public static final double kPositionConversionFactor = 1.0 / 36.0;
    public static final double kEncoderCloseToZero = 2.0;

    // Switch Channel
    public static final int kLimitSwitchChannel = 3;

    // Modes
    private ControlMode m_controlMode = ControlMode.UNSET;
    private InputMode   m_inputMode   = InputMode.NOMINAL;

    // One Pivot motor xjc
    private final CANSparkMax        m_shooterPivotSparkMax;

    private final RelativeEncoder    m_shooterPivotEncoder;

    private final SparkPIDController m_shooterPivotPIDController;

    private double m_setPointPos = 0.0;

    // Limit
    DigitalInput m_limitSwitch = new DigitalInput( kLimitSwitchChannel );

    public ShooterPivot()
    {

        // Pivot
        m_shooterPivotSparkMax = new CANSparkMax(kShooterPivotCanID, MotorType.kBrushless);

        m_shooterPivotEncoder = m_shooterPivotSparkMax.getEncoder();
        m_shooterPivotEncoder.setPositionConversionFactor( kPositionConversionFactor );
        m_shooterPivotEncoder.setPosition( 0.0 );
        
        m_shooterPivotPIDController = m_shooterPivotSparkMax.getPIDController();

        m_shooterPivotPIDController.setFeedbackDevice( m_shooterPivotEncoder );

        m_shooterPivotPIDController.setP(kP);
        m_shooterPivotPIDController.setI(kI);
        m_shooterPivotPIDController.setD(kD);

        m_shooterPivotPIDController.setOutputRange(-1.0, 1.0);
    }

    public InputMode getInputMode()
    {
        return m_inputMode;
    }

    public void slew( double speed, boolean positiveDirection )
    {
        dataLog.publish( "speed", speed );
        dataLog.publish( "posDir", positiveDirection );

        double  encoderPos        = m_shooterPivotEncoder.getPosition();
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

            m_shooterPivotSparkMax.set( speed );

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

            m_shooterPivotPIDController.setReference( m_setPointPos, CANSparkMax.ControlType.kPosition );
        }

    }

    public void home( double speed )
    {
        double driveDownSpeed = -1.0 * Math.abs( speed );

        if ( !m_limitSwitch.get() )
        {
            m_shooterPivotSparkMax.set( driveDownSpeed );
        }
        else
        {
            m_shooterPivotEncoder.setPosition( 0.0 );
            m_inputMode = InputMode.LOWER_LIMIT;
        }

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
