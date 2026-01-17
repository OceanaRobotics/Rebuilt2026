package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

public class AlgaeRemover extends SubsystemBase {
    public static SparkMax algaeMotor = new SparkMax(12, MotorType.kBrushless);
    public static RelativeEncoder algaeEncoder = algaeMotor.getEncoder();

    public void AlgaeMotorSpeed(double algaeInput) {
        algaeMotor.set(algaeInput);
        Shuffleboard.getTab("Algae Motor").addDouble("Algae Encoder Position", () -> algaeEncoder.getPosition());
    }
    
}
