package frc.robot.subsystems;

import java.util.Map;

import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.logging.DataNetworkTableLog;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
// import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkPIDController;

public class Elevator extends SubsystemBase
{

    private static final DataNetworkTableLog dataLog =
    new DataNetworkTableLog( 
        "Subsystems.ElevatorLift",
        Map.of( "speed", DataNetworkTableLog.COLUMN_TYPE.DOUBLE ) );

    // Can IDs
    public static final int kElevatorRightCanID = 12;
    public static final int kElevatorLeftCanID = 13;

    // PID
    private static final double kP = 0.1;
    private static final double kI = 0.0;
    private static final double kD = 0.0;

    private static final double SHOOTER_SLEW_RATE_LIMIT = 0.1;

    private final SlewRateLimiter m_slewRateLimiter = new SlewRateLimiter( SHOOTER_SLEW_RATE_LIMIT );

    private double holdPosition = -1.0;

    // Constants
    public static final double kPositionConversionFactor = 1.0/25.0;
    public static final double kPullyDiamaterM = 38.82/1000;
    
    // Two Motors
    private final CANSparkMax m_elevatorRightSparkMax;
    private final CANSparkMax m_elevatorLeftSparkMax;

    private final RelativeEncoder m_elevatorLiftEncoder;

    private final SparkPIDController m_elevatorPIDController;

    // private final SparkPIDController m_elevatorPIDController;

    public Elevator()
    {
        
        m_elevatorRightSparkMax = new CANSparkMax(kElevatorRightCanID, MotorType.kBrushless);
        m_elevatorLeftSparkMax = new CANSparkMax(kElevatorLeftCanID, MotorType.kBrushless);

        m_elevatorLeftSparkMax.follow(m_elevatorRightSparkMax, true);

        m_elevatorLiftEncoder = m_elevatorRightSparkMax.getEncoder();
        m_elevatorLiftEncoder.setPositionConversionFactor(kPositionConversionFactor);

        // Wheel
        
        m_elevatorPIDController = m_elevatorRightSparkMax.getPIDController();

        m_elevatorPIDController.setFeedbackDevice( m_elevatorLiftEncoder );

        m_elevatorPIDController.setP(kP);
        m_elevatorPIDController.setI(kI);
        m_elevatorPIDController.setD(kD);

        m_elevatorPIDController.setOutputRange(-1.0, 1.0);

    }

    public void roll( double desiredSpeed )
    {
        dataLog.publish( "desiredSpeed", desiredSpeed );

        if ( desiredSpeed != 0.0 )
        {
            double commandedSpeed = m_slewRateLimiter.calculate( desiredSpeed );
            
            dataLog.publish( "commandedSpeed", commandedSpeed );

            m_elevatorRightSparkMax.set( commandedSpeed );

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
                holdPosition = m_elevatorLiftEncoder.getPosition();
                dataLog.publish( "holdPosition", holdPosition );
            }

            m_elevatorPIDController.setReference( holdPosition, CANSparkMax.ControlType.kPosition );
        }



    }

    public void lift( double speed )
    {

        dataLog.publish( "speed", speed );

        m_elevatorRightSparkMax.set(speed);

    }

}
