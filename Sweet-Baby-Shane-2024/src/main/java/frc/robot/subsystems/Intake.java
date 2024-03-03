package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {

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

    public void intake( double speed)
    {

        m_intakeWheelSparkMax.set( speed );

    }

}
