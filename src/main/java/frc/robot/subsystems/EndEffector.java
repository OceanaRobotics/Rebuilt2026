package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;
import au.grapplerobotics.interfaces.LaserCanInterface.RangingMode;
import au.grapplerobotics.interfaces.LaserCanInterface.RegionOfInterest;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

//Extend SubsystemBase for use with Commands
public class EndEffector extends SubsystemBase{
    //Set up LaserCan and EndEffector motor
    private static LaserCan laserCan = new LaserCan(22);
    private SparkMax endEffector = new SparkMax(11, MotorType.kBrushless);

    //Intake Motor Function
    public void intake(double input) {
        endEffector.set(input);
    }
    
    //Distance Reading From LaserCan
    public static double laserCanDist() {
        return laserCan.getMeasurement().distance_mm;
    }
}
