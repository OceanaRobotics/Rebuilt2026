package frc.robot.subsystems.swervedrive;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

//Extend SubsystemBase to use with commands
public class Elevator extends SubsystemBase{
    //Setup Elevator motor and encoder
    public static SparkMax elevatorVert = new SparkMax(9, MotorType.kBrushless);
    public static RelativeEncoder encoder = elevatorVert.getEncoder();

    public void vertDrive(double input) {
        elevatorVert.set(input);
    }
    
}
