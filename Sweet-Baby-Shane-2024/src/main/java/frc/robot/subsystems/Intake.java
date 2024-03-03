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
            Map.of( "speed", DataNetworkTableLog.COLUMN_TYPE.DOUBLE ) );

    // Can IDs
    public static final int kIntakeWheelCanID = 14;
    public static final int kIntakePivotCanID = 15;

    // 2 Motors
    private final CANSparkMax m_intakeWheelSparkMax;
    private final CANSparkMax m_intakePivotSparkMax;

    private final RelativeEncoder m_intakeWheelEncoder;
    private final RelativeEncoder m_intakePivotEncoder;

    public Intake()
    {

        m_intakeWheelSparkMax = new CANSparkMax(kIntakeWheelCanID, MotorType.kBrushless);
        m_intakePivotSparkMax = new CANSparkMax(kIntakePivotCanID, MotorType.kBrushless);

        m_intakeWheelEncoder = m_intakeWheelSparkMax.getEncoder();
        m_intakePivotEncoder = m_intakePivotSparkMax.getEncoder();

    }

    public void intake(double speed)
    {
        dataLog.publish( "speed", speed );

        m_intakeWheelSparkMax.set( speed );

    }

}
