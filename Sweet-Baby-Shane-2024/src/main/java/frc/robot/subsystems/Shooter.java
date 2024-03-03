package frc.robot.subsystems;

import java.util.Map;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.logging.DataNetworkTableLog;

// 4 Motors
// private final CANSparkMax m_shooterFlywheelOneSparkMax;
// private final CANSparkMax m_shooterFlywheelTwoSparkMax;
// private final CANSparkMax m_shooterPivotSparkMax;
// private final CANSparkMax m_shooterFeederSparkMax;

// private final RelativeEncoder m_shooterFlywheelOneEncoder;
// private final RelativeEncoder m_shooterFlywheelTwoEncoder;
// private final RelativeEncoder m_shooterPivotEncoder;
// private final RelativeEncoder m_shooterFeederEncoder;

// private final SparkPIDController m_shooterFlywheelOnePIDController;
// private final SparkPIDController m_shooterFlywheelPIDController;
// private final SparkPIDController m_shooterPivotPIDController;
// private final SparkPIDController m_shooterFeederPIDController;

public class Shooter extends SubsystemBase
{
    private static final DataNetworkTableLog dataLog =
        new DataNetworkTableLog( 
            "Subsystems.Shooter",
            Map.of( "speed", DataNetworkTableLog.COLUMN_TYPE.DOUBLE ) );

    // Can IDs
    public static final int kShooterFlywheelRightCanID = 16;
    public static final int kShooterFlywheelLeftCanID = 17;
    public static final int kShooterPivotCanID = 18;
    public static final int kShooterFeedCanID = 19;

    // Two fly-wheel motors
    private final CANSparkMax m_shooterFlywheelRightSparkMax;
    private final CANSparkMax m_shooterFlywheelLeftSparkMax;

    private final RelativeEncoder m_shooterFlywheelEncoder;

    // One Pivot motor
    private final CANSparkMax m_shooterPivotSparkMax;

    private final RelativeEncoder m_shooterPivotEncoder;

    // One Feed motor
    private final CANSparkMax m_shooterFeedSparkMax;

    private final RelativeEncoder m_shooterFeedEncoder;


    public Shooter()
    {

        // Flywheel
        m_shooterFlywheelRightSparkMax = new CANSparkMax(kShooterFlywheelRightCanID, MotorType.kBrushless);
        m_shooterFlywheelLeftSparkMax = new CANSparkMax(kShooterFlywheelLeftCanID, MotorType.kBrushless);

        m_shooterFlywheelLeftSparkMax.follow(m_shooterFlywheelRightSparkMax, true);

        m_shooterFlywheelEncoder = m_shooterFlywheelRightSparkMax.getEncoder();

        // Pivot
        m_shooterPivotSparkMax = new CANSparkMax(kShooterPivotCanID, MotorType.kBrushless);
        m_shooterPivotEncoder = m_shooterPivotSparkMax.getEncoder();

        // Feed
        m_shooterFeedSparkMax = new CANSparkMax(kShooterFeedCanID, MotorType.kBrushless);
        m_shooterFeedEncoder = m_shooterFeedSparkMax.getEncoder();
    }

    public void spin( double speed )
    {

        dataLog.publish( "speed", speed );

        m_shooterFlywheelRightSparkMax.set(speed);

    }

}
