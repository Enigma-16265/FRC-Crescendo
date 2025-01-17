package frc.robot.subsystems;

import java.util.Map;

import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

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
                "simEncoderPos", DataNetworkTableLog.COLUMN_TYPE.DOUBLE,
                "homeMode", DataNetworkTableLog.COLUMN_TYPE.STRING ) );

    // Can IDs
    public static final int kShooterPivotCanID = 12;
    
    // PID
    private static final double kP = 0.0;
    private static final double kI = 0.0;
    private static final double kD = 0.0;

    // Constants
    public static final double kPositionConversionFactor = 1.0 / 36.0;
    public static final double kEncoderCloseToZero = 2.0;

    // Switch Channel
    public static final int kLimitSwitchChannel = 2;

    // Modes
    private ControlMode m_controlMode = ControlMode.UNSET;
    private InputMode   m_inputMode   = InputMode.NOMINAL;

    // Motor
    private final SparkFlex m_shooterPivotSparkFlex;

    private final RelativeEncoder m_shooterPivotEncoder;

    private final SparkClosedLoopController m_shooterPivotPIDController;

    // Limit
    DigitalInput m_limitSwitch = new DigitalInput( kLimitSwitchChannel );

    private double m_setPointPos = 0.0;

    public final SparkFlexConfig m_shooterPivotSparkFlexConfig;

    public ShooterPivot()
    {

        m_shooterPivotSparkFlexConfig = new SparkFlexConfig();
        m_shooterPivotSparkFlexConfig.idleMode( IdleMode.kBrake );
        m_shooterPivotSparkFlexConfig.encoder.positionConversionFactor( kPositionConversionFactor );
        m_shooterPivotSparkFlexConfig.closedLoop.pid(kP, kI, kD);
        m_shooterPivotSparkFlexConfig.closedLoop.outputRange( -1.0, 1.0 );

        // Pivot
        m_shooterPivotSparkFlex = new SparkFlex(kShooterPivotCanID, MotorType.kBrushless);

        m_shooterPivotEncoder = m_shooterPivotSparkFlex.getEncoder();
        m_shooterPivotEncoder.setPosition( 0.0 );
        
        m_shooterPivotPIDController = m_shooterPivotSparkFlex.getClosedLoopController();
    }

    public InputMode getInputMode()
    {
        return m_inputMode;
    }

    public void slew( double speed, boolean positiveDirection )
    {
        // dataLog.publish( "speed", speed );
        // dataLog.publish( "posDir", positiveDirection );

        double  encoderPos        = m_shooterPivotEncoder.getPosition();
        boolean limitSwitchActive = !m_limitSwitch.get();

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

            m_shooterPivotPIDController.setReference( speed, SparkMax.ControlType.kDutyCycle );

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
            m_shooterPivotPIDController.setReference( 0.0, SparkMax.ControlType.kDutyCycle );
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

            m_shooterPivotPIDController.setReference( driveDownSpeed, SparkMax.ControlType.kDutyCycle );

            if ( RobotBase.isSimulation() )
            {
                updateSimEncoder( speed );
            }
        }
        else
        {
            m_homeMode = HomeMode.HOMED;
            m_inputMode = InputMode.LOWER_LIMIT;
            m_shooterPivotEncoder.setPosition( 0.0 );
            m_shooterPivotPIDController.setReference( 0.0, SparkMax.ControlType.kPosition );
        }

        // dataLog.publish( "homeMode", m_homeMode );

        double  encoderPos = m_shooterPivotEncoder.getPosition();
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
