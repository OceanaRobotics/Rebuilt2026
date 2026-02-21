package frc.robot.subsystems.swervedrive;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.config.SparkMaxConfig;

public class Intake extends SubsystemBase {
    private SparkMax intakeMotor = new SparkMax(12, MotorType.kBrushless);
    private SparkClosedLoopController intakeController = intakeMotor.getClosedLoopController();
    private SparkMaxConfig intakeConfig = new SparkMaxConfig();
    private RelativeEncoder intakeEncoder = intakeMotor.getEncoder();
    private SparkMax intakeMover = new SparkMax(13, MotorType.kBrushless);

    public Intake() {
        intakeConfig.encoder.positionConversionFactor(1).velocityConversionFactor(1);
        intakeConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .p(0)
            .i(0)
            .d(0)
            .outputRange(0, 1)
            .feedForward.kV(0, ClosedLoopSlot.kSlot0);

        intakeMotor.configure(intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("intake rpm: ", intakeEncoder.getVelocity());
    }

    /**
     * Run the intake at a given percent
     * @param percent
     * @return A {@link RunCommand}
     */
    public Command runSystemAtPercent(double percent) {
        return run(() -> {
            intakeMotor.set(percent);
        });
    }

    /**
     * Run the intake at a desired RPM
     * @param rpm
     * @return A {@link RunCommand}
     */
    public Command runSystemAtVelocity(double rpm) {
        return run(() -> {
            intakeController.setSetpoint(rpm, ControlType.kVelocity);
        });
    }

    /**
     * Stop the intake system
     * @return A {@link RunCommand}
     */
    public Command stopSystem() {
        return run(() -> {
            intakeController.setSetpoint(0, ControlType.kVelocity);
            intakeMover.set(0);
        });
    }

    /**
     * Extend out the intake arm
     * @return A {@link Runcommand}
     */
    public Command extendIntake() {
        return run(() -> {
            intakeMover.set(0.5);
        }).withTimeout(1).andThen(stopSystem());
    }

    /**
     * Retract the intake arm
     * @return A {@link Runcommand}
     */
    public Command retractIntake() {
        return run(() -> {
            intakeMover.set(-0.5);
        }).withTimeout(1).andThen(stopSystem());
    }
}