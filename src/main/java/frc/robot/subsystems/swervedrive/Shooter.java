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

public class Shooter extends SubsystemBase {
    private SparkMax shooterMotor = new SparkMax(11, MotorType.kBrushless);
    private SparkClosedLoopController motorController = shooterMotor.getClosedLoopController();
    private SparkMaxConfig motorConfig = new SparkMaxConfig();
    private RelativeEncoder motorEncoder = shooterMotor.getEncoder();
    private Hopper m_hopper = new Hopper();

    public Shooter() {
        motorConfig.encoder.positionConversionFactor(1).velocityConversionFactor(1);
        motorConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .p(0.000150)
            .i(0)
            .d(0)
            .outputRange(0, 1)
            .feedForward.kV(0.000150, ClosedLoopSlot.kSlot0);

        shooterMotor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("shooter rpm: ", motorEncoder.getVelocity());
        SmartDashboard.putNumber("shooter set: ", shooterMotor.get());
        SmartDashboard.putNumber("shooter setpoint: ", motorController.getSetpoint());
    }

    /**
     * Run the shooter motor at a percent power
     * @param power The desired power percent to run the motor at
     * @return A {@link RunCommand}
     */
    public Command runSystemAtPercent() {
        return run(() -> {
            shooterMotor.set(SmartDashboard.getNumber("shooter power: ", 0.2));
        });
    }

    /**
     * Ends ONLY the shooter system, call {@link #stopFullSystem()} to end the hopper system
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
    public Command runSystemAtVelocity() {
        return run(() -> {
            motorController.setSetpoint(SmartDashboard.getNumber("desired rpm: ", 0), ControlType.kVelocity);
        });
    }

    /**
     * !! UNFINISHED !!
     * <li>!! UNTESTED !!</li>
     * Run the entire shooter system and attempt to score
     * @return A very fancy {@link RunCommand}
     */
    public Command runShooterSystem() {
        return run(() -> {
            m_hopper.runSystemAtVelocity(480, 1500)
            .withTimeout(0.3)
            .andThen(runSystemAtVelocity());
        });
    }

    /**
     * Reconfigures the shooter {@link SparkMax} based on {@link SmartDashboard} settings
     * @return A {@link RunCommand}
     */
    public Command reconfigureMotor() {
        return run(() -> {
            motorConfig.encoder.positionConversionFactor(1).velocityConversionFactor(1);
            motorConfig.closedLoop
                    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                    .p(SmartDashboard.getNumber("shooter p: ", 0.000150))
                    .i(SmartDashboard.getNumber("shooter i: ", 0))
                    .d(SmartDashboard.getNumber("shooter d: ", 0))
                    .outputRange(0, 1)
                .feedForward
                    // .kS(SmartDashboard.getNumber("shooter kS: ", 0), ClosedLoopSlot.kSlot0);
                    .kV(SmartDashboard.getNumber("shooter kV: ", 0.000150), ClosedLoopSlot.kSlot0);

            shooterMotor.configure(motorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        }).withTimeout(0.1);
    } 

    /**
     * Ends BOTH the shooter and hopper systems
     * @return A {@link RunCommand}
     */
    public Command stopFullSystem() {
        return run(() -> {
            stopSystem();
            m_hopper.stopSystem();
        });
    }

}
