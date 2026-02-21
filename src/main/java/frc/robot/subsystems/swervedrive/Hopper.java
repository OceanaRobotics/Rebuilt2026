package frc.robot.subsystems.swervedrive;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.RunCommand;

public class Hopper extends SubsystemBase {
    private SparkMax agitatorMotor = new SparkMax(9, MotorType.kBrushless);
    private SparkMax kickerMotor = new SparkMax(10, MotorType.kBrushless);
    private SparkClosedLoopController agitatorController = agitatorMotor.getClosedLoopController();
    private SparkClosedLoopController kickerController = kickerMotor.getClosedLoopController();
    private SparkMaxConfig agitatorConfig = new SparkMaxConfig();
    private SparkMaxConfig kickerConfig = new SparkMaxConfig();
    private RelativeEncoder agitatorEncoder = agitatorMotor.getEncoder();
    private RelativeEncoder kickerEncoder = kickerMotor.getEncoder();
    private double reverseRPM = -200;

    public Hopper() {
        agitatorConfig.encoder.positionConversionFactor(1).velocityConversionFactor(1);
        agitatorConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .p(0)
            .i(0)
            .d(0)
            .outputRange(0, 1)
        .feedForward
            .kV(0, ClosedLoopSlot.kSlot0);
        agitatorMotor.configure(agitatorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        kickerConfig.encoder.positionConversionFactor(1).velocityConversionFactor(1);
        kickerConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .p(0)
            .i(0)
            .d(0)
            .outputRange(0, 1)
        .feedForward
            .kV(0, ClosedLoopSlot.kSlot0);
        kickerMotor.configure(kickerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("agitator rpm: ", agitatorEncoder.getVelocity());
        SmartDashboard.putNumber("kicker rpm: ", kickerEncoder.getVelocity());
    }

    /**
     * Run the hopper and kicker system
     * @param agitatorPercent
     * @param kickerPercent
     * @return A {@link RunCommand}
     */
    public Command runSystemAtPercent(double agitatorPercent, double kickerPercent) {
        return run(() -> {
            agitatorMotor.set(agitatorPercent);
            kickerMotor.set(kickerPercent);
        });
    }

    /**
     * Run the hopper and kicker system in reverse to unclog jams
     * @return A {@link RunCommand}
     */
    public Command reverseSystem() {
        return run(() -> {
            agitatorController.setSetpoint(reverseRPM, ControlType.kVelocity);
            kickerController.setSetpoint(reverseRPM, ControlType.kVelocity);
        });
    }
    /**
     * Stop the hopper and kicker
     * @return A {@link RunCommand}
     */
    public Command stopSystem() {
        return run(() -> {
            agitatorController.setSetpoint(0, ControlType.kVelocity);
            kickerController.setSetpoint(0, ControlType.kVelocity);
        });
    }
    /**
     * Run the hopper and kicker system at a desired RPM
     * @param agitatorRPM
     * @param kickerRPM
     * @return A {@link RunCommand}
     */
    public Command runSystemAtVelocity(double agitatorRPM, double kickerRPM) {
        return run(() -> {
            agitatorController.setSetpoint(agitatorRPM, ControlType.kVelocity);
            kickerController.setSetpoint(kickerRPM, ControlType.kVelocity);
        });
    }
}
