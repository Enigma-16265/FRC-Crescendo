package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase 
{

    private CANSparkMax motorController;
    private RelativeEncoder encoder;
    private PIDController pidController;

    private static final double ANGLE_TOLERANCE = 2.0; // degrees
    private static final double maxAngle = 90.0; // maximum angle
    private static final double minAngle = 0.0; // minimum angle
    
    private static final double KP = 0.1;
    private static final double KI = 0.0;
    private static final double KD = 0.0;

    public ShooterSubsystem() {
        motorController = new CANSparkMax(1, MotorType.kBrushless); // Example CAN ID
        encoder = motorController.getEncoder(); // Get encoder from motor controller
        encoder.setPositionConversionFactor(1.0 / 36.0); // Set conversion factor to 1/36

        // Initialize PID controller with constants
        pidController = new PIDController(KP, KI, KD); // Example PID constants
    }

    public void setTargetAngle(double angle) {
        // Ensure angle is within limits
        double targetAngle = Math.max(Math.min(angle, maxAngle), minAngle);

        // Set PID setpoint
        pidController.setSetpoint(targetAngle);
    }

    @Override
    public void periodic() {
        // Get current angle from encoder
        double currentAngle = encoder.getPosition();
        
        // Calculate PID output and directly set the motor output
        motorController.set(pidController.calculate(currentAngle));
    }

}
