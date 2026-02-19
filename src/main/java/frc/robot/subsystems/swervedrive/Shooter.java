package frc.robot.subsystems.swervedrive;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.config.SparkMaxConfig;

public class Shooter extends SubsystemBase {
    private SparkMax shooterMotor = new SparkMax(10, MotorType.kBrushless);
    private SparkClosedLoopController motorController = shooterMotor.getClosedLoopController();
    private SparkMaxConfig motorConfig = new SparkMaxConfig();

    public Shooter() {
        motorConfig.encoder.positionConversionFactor(1).velocityConversionFactor(1);
        motorConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .p(0)
            .i(0)
            .d(0)
            .outputRange(0, 1)
            .feedForward.kV(12.0 / 5767, ClosedLoopSlot.kSlot1);

        shooterMotor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }
    /**
     * Run the shooter motor at a percent power
     * @param power The desired power percent to run the motor at
     * @return A {@link RunCommand}
     */
    public Command runShooterMotor(double power) {
        return run(() -> {
            shooterMotor.set(power);
        });
    }

    /**
     * Stops the shooter motor
     * @return A {@link RunCommand}
     */
    public Command stopSystem() {
        return run(() -> {
            shooterMotor.set(0);
        });
    }

    /**
     * Run the shooter in velocity control mode
     * @param rpm The desired RPM to run nthe motor at
     * @return A {@link RunCommand}
     */
    public Command runAtVelocity(double rpm) {
        return run(() -> {
            motorController.setSetpoint(rpm, ControlType.kVelocity);
        });
    }

    /**
     * !! UNFINISHED !!
     * Run the entire shooter system and attempt to score
     * @return A very fancy {@link RunCommand}
     */
    public Command runShooterSystem() {
        return run(() -> {
            System.out.println("unga bunga");
        });
    }

}
