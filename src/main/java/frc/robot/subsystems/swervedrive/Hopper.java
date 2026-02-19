package frc.robot.subsystems.swervedrive;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.RunCommand;

public class Hopper extends SubsystemBase {
    private SparkMax agitator = new SparkMax(9, MotorType.kBrushless);
    private SparkMax kicker = new SparkMax(10, MotorType.kBrushless);

    /**
     * Run the hopper and kicker system
     * @return A {@link RunCommand}
     */
    public Command runSystem() {
        return run(() -> {
            agitator.set(0.25);
            kicker.set(0.5);
        });
    }

    /**
     * Run the hopper and kicker system in reverse to unclog jams
     * @return A {@link RunCommand}
     */
    public Command reverseSystem() {
        return run(() -> {
            agitator.set(-0.1);
            kicker.set(-0.1);
        });
    }
    /**
     * Stop the hopper and kicker
     * @return A {@link RunCommand}
     */
    public Command stopSystem() {
        return run(() -> {
            agitator.set(0);
            kicker.set(0);
        });
    }
    
}
