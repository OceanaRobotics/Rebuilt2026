package frc.robot.subsystems.swervedrive;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;

public class Climber extends SubsystemBase{
    private SparkMax climberMotor = new SparkMax(14, MotorType.kBrushless);
    
    /**
     * Run the climber at a given percent
     * @param percent
     * @return A {@link RunCommand}
     */
    public Command runSystemAtPercent(double percent) {
        return run(() -> {
            climberMotor.set(percent);
        });
    }

    /**
     * Stop the climber system
     * @return A {@link RunCommand}
     */
    public Command stopSystem() {
        return run(() -> {
            climberMotor.set(0);
        });
    }
    
}
