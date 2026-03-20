package frc.robot.subsystems.swervedrive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
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
    public Hopper m_hopper = new Hopper();
    public final Transform2d shooterOffset = new Transform2d(Units.inchesToMeters(-9.1), Units.inchesToMeters(-1.4), new Rotation2d(Units.degreesToRadians(90)));
    public Double distanceToHub;

  /**
   * A subsystem handling the entire shooting pipeline, including hopper and intake
   */
  public Shooter() {
    motorConfig.encoder.positionConversionFactor(1).velocityConversionFactor(1);
    motorConfig.closedLoop
      .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      .p(0.0002)
      .i(0)
      .d(0.005)
      .outputRange(0, 1)
      .feedForward.kV(0.000195, ClosedLoopSlot.kSlot0);
    shooterMotor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("shooter rpm: ", motorEncoder.getVelocity());
  }

  /**
   * Run the shooter motor at a percent power
   * @param power The desired power percent to run the motor at
   * @return {@link RunCommand} - Command to run
   */
  public Command runSystemAtPercent() {
    return run(() -> {
      shooterMotor.set(SmartDashboard.getNumber("shooter power: ", 0.2));
    });
  }

  /**
   * Ends ONLY the shooter system, call {@link #stopFullSystem()} to end the hopper system
   * @return {@link RunCommand} - Command to run
   */
  public Command stopSystem() {
    return run(() -> {
      shooterMotor.set(0);
    });
  }

  /**
   * Run the shooter in velocity control mode
   * @param rpm The desired RPM to run the motor at
   * @return {@link RunCommand} - Command to run
   */
  public Command runSystemAtVelocity(SwerveSubsystem drivebase) {
    return run(() -> {
      motorController.setSetpoint(this.getOptimalShooterVelocity(drivebase), ControlType.kVelocity);
    });
  }

  /**
   * Run the entire shooter system and attempt to score
   * @param dt - The robot drivetrain
   * @return {@link RunCommand} - Very fancy command to run
   */
  public Command runShooterSystem(SwerveSubsystem dt) {
    return runSystemAtVelocity(dt).withTimeout(0.1).andThen(
        aimAtClosestHub(dt)
      ).andThen(
        m_hopper.runSystemAtPercent(0.5, 0.5).withTimeout(0.1)
      );
  }

  /**
   * Reconfigures the shooter {@link SparkMax} based on {@link SmartDashboard} settings
   * <li> !! DO NOT CALL OUTSIDE OF TESTING !! </li>
   * @return {@link RunCommand} - Command to run
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
          .kV(SmartDashboard.getNumber("shooter kV: ", 0.000150), ClosedLoopSlot.kSlot0);
      shooterMotor.configure(motorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }).withTimeout(0.1);
  } 

  /**
   * Ends BOTH the shooter and hopper systems
   * @return {@link RunCommand} - Command to run
   */
  public Command stopFullSystem() {
    return stopSystem().withTimeout(0.1).andThen(
        m_hopper.stopSystem().withTimeout(0.1)
      );
  }

  /**
  * Aim the shooter at the hub of the current alliance
  * @param dt - The robot drivetrain
  * @return {@link RunCommand} - Command to run
  */
  public Command aimAtHub(SwerveSubsystem dt) {
    return run(() -> {
      Pose2d currentPose = getShooterPose(dt);
      Pose2d hubPose = dt.isRedAlliance() ? new Pose2d(new Translation2d(Units.inchesToMeters(651.22 - 182.11), Units.inchesToMeters(158.84)), new Rotation2d(0)) : new Pose2d(new Translation2d(Units.inchesToMeters(182.11), Units.inchesToMeters(158.84)), new Rotation2d(0));
      Translation2d difference = currentPose.relativeTo(hubPose).getTranslation();
      Rotation2d vector = new Rotation2d(difference.getX(), difference.getY());
      dt.drive(ChassisSpeeds.fromRobotRelativeSpeeds(0, 0, ((dt.getPose().getRotation().getDegrees() - 90) - vector.getDegrees()) * 0.055, dt.getHeading()));
    }).withTimeout(1);
  }

  /**
   * Aim the shooter at whichever hub is closest, regardless of alliance
   * @param dt - The robot drivetrain
   * @return {@link RunCommand} - Command to run
   */
  public Command aimAtClosestHub(SwerveSubsystem dt) {
    return run(() -> {
      Pose2d currentPose = dt.getPose();
      Pose2d blueHubPose = new Pose2d(new Translation2d(Units.inchesToMeters(182.11), Units.inchesToMeters(158.84)), new Rotation2d(0));
      Pose2d redHubPose = new Pose2d(new Translation2d(Units.inchesToMeters(651.22 - 182.11), Units.inchesToMeters(158.84)), new Rotation2d(0));
      Translation2d differenceToBlueHub = currentPose.relativeTo(blueHubPose).getTranslation();
      Translation2d differenceToRedHub = currentPose.relativeTo(redHubPose).getTranslation();
      Double distanceToBlueHub = differenceToBlueHub.getNorm();
      Double distanceToRedHub = differenceToRedHub.getNorm();
      boolean closerToRedHub = distanceToRedHub > distanceToBlueHub;
      if (closerToRedHub) {
        distanceToHub = distanceToRedHub;
        Rotation2d vector = new Rotation2d(differenceToRedHub.getX(), differenceToRedHub.getY());
        dt.drive(ChassisSpeeds.fromRobotRelativeSpeeds(0, 0, ((dt.getPose().getRotation().getDegrees() - 90) - vector.getDegrees()) * 0.055, dt.getHeading()));
      } else {
        distanceToHub = distanceToBlueHub;
        Rotation2d vector = new Rotation2d(differenceToBlueHub.getX(), differenceToBlueHub.getY());
        dt.drive(ChassisSpeeds.fromRobotRelativeSpeeds(0, 0, ((dt.getPose().getRotation().getDegrees() - 90) - vector.getDegrees()) * 0.055, dt.getHeading()));
      }
    }).withTimeout(1);
  }

  /**
   * Get the current straight-line distance to the center of the hub (purely X and Y, height is not considered)
   * @param dt - The robot drivetrain
   * @return {@link Double} - Distance to the hub
   */
  public double getDistanceToHub(SwerveSubsystem dt, boolean ignoreAlliance) {
    if (ignoreAlliance) {
      return distanceToHub;
    }
    Pose2d currentPose = dt.getPose();
    Pose2d hubPose = dt.isRedAlliance() ? new Pose2d(new Translation2d(Units.inchesToMeters(651.22 - 182.11), Units.inchesToMeters(158.84)), new Rotation2d(0)) : new Pose2d(new Translation2d(Units.inchesToMeters(182.11), Units.inchesToMeters(158.84)), new Rotation2d(0));
    Translation2d difference = currentPose.relativeTo(hubPose).getTranslation();
    return difference.getNorm();
  }

  /**
   * Get the pose of the shooter on the field, rather than the center of the robot
   * @param dt - The robot drivetrain
   * @return {@link Pose2d} - The shooter pose relative to the field
   */
  public Pose2d getShooterPose(SwerveSubsystem dt) {
    return dt.getPose().plus(shooterOffset);
  }

  /**
   * A simple command to wait for a specified duration
   * @param duration - Desired time to wait in seconds
   * @return {@link WaitCommand} - Command to run
   */
  public Command waitDuration(double duration) {
    return new WaitCommand(duration);
  }

  /**
   * Compute the optimal velocity for the shooter at the current distance to score
   * <li> !! PROPER SHOOTER EQUATION NOT INPUT !! </li>
   * @param dt - The robot drivetrain
   * @return {@link Double} - RPM
   */
  public double getOptimalShooterVelocity(SwerveSubsystem dt) {
    double distance = this.getDistanceToHub(dt, true);
    double rpm = (567.3692 * distance) + 2419.8747;
    return rpm;
  }

}
