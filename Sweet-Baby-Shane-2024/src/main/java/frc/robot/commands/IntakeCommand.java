package frc.robot.commands;

import java.util.Map;
import java.util.function.Supplier;

import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.logging.DataNetworkTableLog;
import frc.robot.Robot;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.IntakePivot;
import edu.wpi.first.wpilibj.I2C;

public class IntakeCommand extends Command 
{
    private static final DataNetworkTableLog dataLog =
    new DataNetworkTableLog( 
        "Subsystems.Intake.DefaultCommand",
        Map.of( "color", DataNetworkTableLog.COLUMN_TYPE.STRING ) );
    
    // Color Commands
    private final ColorSensorV3 colorSensor;
    private final ColorMatch colorMatcher;

    private static final double kRollInSlewRateLimit = 0.3;
    private static final double kRollOutSlewRateLimit = 1;

    private final Intake m_intake;
    private final Supplier<Double> m_speedSupplier;
    private final SlewRateLimiter  m_inSlewRateLimiter = new SlewRateLimiter( kRollInSlewRateLimit );
    private final SlewRateLimiter  m_outSlewRateLimiter = new SlewRateLimiter( kRollOutSlewRateLimit );

    private final IntakePivot m_intakePivot;

    public IntakeCommand(
        Intake intake,
        Supplier<Double> speedSupplier,
        IntakePivot intakePivot )
    {
        this.m_intake = intake;
        this.m_speedSupplier = speedSupplier;
        this.m_intakePivot = intakePivot;

            // Initialize the color sensor on I2C port 0 (change as necessary)
    colorSensor = new ColorSensorV3(I2C.Port.kOnboard);
    //private final ColorSensorV3 colorSensor = new ColorSensorV3( I2C.Port.kOnboard );
        
    // Color matcher target color
    colorMatcher = new ColorMatch();
    colorMatcher.addColorMatch(Color.kOrange);

        addRequirements(intake);
    }

    @Override
    public void initialize() {}
    
    @Override
    public void execute()
    {
        double requestSpeed = m_speedSupplier.get();

        double commandSpeed = 0;

        if ( requestSpeed > 0.0 )
        {
        
            // commandSpeed = m_outSlewRateLimiter.calculate( requestSpeed );
            commandSpeed = 1.0;
        
        } else if ( requestSpeed < 0.0 )
        {

            // commandSpeed = m_inSlewRateLimiter.calculate( requestSpeed );
            commandSpeed = -0.6;

        }
        else
        {
            m_outSlewRateLimiter.reset( 0.0 );
            m_inSlewRateLimiter.reset( 0.0 );
        }

        m_intake.roll( commandSpeed );

        colorSensor();
    }

    @Override
    public void end( boolean interrupted ) {}

    @Override
    public boolean isFinished() {
        return false;
    }

    private int simCollects = 0;
    public void colorSensor()
    {

        // System.out.println("Color Sensor");
        Color detectedColor = colorSensor.getColor();
        // dataLog.publish( "color",rgbToHex(detectedColor.red, detectedColor.green, detectedColor.blue) );
            
        // Perform color matching
        ColorMatchResult match = colorMatcher.matchClosestColor(detectedColor);

        // Execute command if orange is detected
        boolean colorMatched = (match.color == Color.kOrange);

        if ( Robot.isSimulation() )
        {
            if ( ( simCollects > 3000 ) && ( colorMatched == false ) )
            {
                colorMatched = true;
            }

            simCollects++;
        }

        if (colorMatched)
        {

            System.out.println("Detected Orange");
            Command sensorIntakePivot = new IntakePivotLimitCommand( m_intakePivot, IntakePivotLimitCommand.Behavior.STOW );
            sensorIntakePivot.schedule();
        }

    }

  public String rgbToHex(double red, double green, double blue)
  {
    int r = (int) (red * 255);
    int g = (int) (green * 255);
    int b = (int) (blue * 255);
    return String.format("#%02X%02X%02X", r, g, b);
  }

}
