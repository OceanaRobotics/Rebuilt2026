// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.swervedrive.Hopper;
import frc.robot.subsystems.swervedrive.Intake;
import frc.robot.subsystems.swervedrive.Shooter;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.io.File;
import swervelib.SwerveInputStream;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{
  // Setup subsystems here for use with commands later
  // Replace with CommandPS4Controller or CommandJoystick if needed
  public final CommandXboxController driverXbox = new CommandXboxController(0);
  public final SwerveSubsystem drivebase  = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve/neo"));
  public final Shooter shooter = new Shooter();
  public final Intake intake = new Intake();
  public final Hopper hopper = shooter.m_hopper;
  public final SendableChooser<Command> autoChooser;

  /**
   * Converts driver input into a field-relative ChassisSpeeds that is controlled by angular velocity.
   */
  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                () -> driverXbox.getLeftY() * -1,
                                                                () -> driverXbox.getLeftX() * -1)
                                                            .withControllerRotationAxis(driverXbox::getRightX)
                                                            .deadband(OperatorConstants.DEADBAND)
                                                            .scaleTranslation(0.8)
                                                            .allianceRelativeControl(true);

  /**
   * Clone's the angular velocity input stream and converts it to a fieldRelative input stream.
   */
  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(driverXbox::getRightX, driverXbox::getRightY).headingWhile(true);

  /**
   * Clone's the angular velocity input stream and converts it to a robotRelative input stream.
   */
  SwerveInputStream driveRobotOriented = driveAngularVelocity.copy().robotRelative(true).allianceRelativeControl(false);

  SwerveInputStream driveAngularVelocityKeyboard = SwerveInputStream.of(drivebase.getSwerveDrive(), () -> -driverXbox.getLeftY(), () -> -driverXbox.getLeftX())
      .withControllerRotationAxis(() -> driverXbox.getRawAxis(2))
      .deadband(OperatorConstants.DEADBAND)
      .scaleTranslation(0.8)
      .allianceRelativeControl(true);

  // Derive the heading axis with math!
  SwerveInputStream driveDirectAngleKeyboard = driveAngularVelocityKeyboard.copy()
      .withControllerHeadingAxis(
        () -> Math.sin(driverXbox.getRawAxis(2) * Math.PI) * (Math.PI * 2),
        () -> Math.cos(driverXbox.getRawAxis(2) * Math.PI) * (Math.PI * 2))
      .headingWhile(true);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer()
  {
    // Shooter auto commands
    NamedCommands.registerCommand("runShooterAtVelocity",  shooter.runSystemAtVelocity(2)); // !! ADJUST THIS !!
    NamedCommands.registerCommand("runShooterSystem",  shooter.runShooterSystem(drivebase));
    NamedCommands.registerCommand("stopFullSystem", shooter.stopFullSystem());
    NamedCommands.registerCommand("aimAtHub", shooter.aimAtHub(drivebase));
    
    // Intake auto commands
    NamedCommands.registerCommand("extendIntake", intake.extendIntake());
    NamedCommands.registerCommand("retractIntake", intake.retractIntake());
    NamedCommands.registerCommand("runIntakeAtVelocity", intake.runSystemAtVelocity(1));
    NamedCommands.registerCommand("stopIntake", intake.stopSystem());

    // Hopper auto commands
    NamedCommands.registerCommand("runHopperAtVelocity", hopper.runSystemAtVelocity(1,1));
    NamedCommands.registerCommand("reverseHopper", hopper.reverseSystem());

    // Add the choices to autoChooser
    autoChooser = AutoBuilder.buildAutoChooser();
    autoChooser.addOption("BlueBasicLeftAuto", new PathPlannerAuto("BlueBasicAuto"));
    autoChooser.addOption("BlueBasicRightAuto", new PathPlannerAuto("BlueBasicAuto" , true));
    autoChooser.addOption("RedBasicRightAuto", new PathPlannerAuto("RedBasicAuto"));
    autoChooser.addOption("RedBasicLeftAuto", new PathPlannerAuto("RedBasicAuto", true));
    autoChooser.addOption("RedCenterAuto", new PathPlannerAuto("RedCenterAuto"));
    autoChooser.addOption("BlueCenterAuto", new PathPlannerAuto("BlueCenterAuto"));

    SmartDashboard.putData("Auto Chooser", autoChooser);

    // Configure the trigger bindings
    configureBindings();
    DriverStation.silenceJoystickConnectionWarning(true);
    NamedCommands.registerCommand("test", Commands.print("I EXIST"));
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary predicate, or via the
   * named factories in {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
   */
  private void configureBindings()
  {
    Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);
    Command driveFieldOrientedDirectAngleKeyboard = drivebase.driveFieldOriented(driveDirectAngleKeyboard);

    if (RobotBase.isSimulation()) {
      drivebase.setDefaultCommand(driveFieldOrientedDirectAngleKeyboard);
      driverXbox.start().onTrue(Commands.runOnce(() -> drivebase.resetOdometry(new Pose2d(3, 3, new Rotation2d()))));
      driverXbox.button(1).whileTrue(drivebase.sysIdDriveMotorCommand());
    } else {
      drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);
    }
    
    if (DriverStation.isTest()) {
      // This is for test mode only
      drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity); // Overrides drive command above!

      driverXbox.x().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
      driverXbox.start().onTrue((Commands.runOnce(drivebase::zeroGyro)));
      driverXbox.back().whileTrue(drivebase.centerModulesCommand());
      driverXbox.leftBumper().onTrue(Commands.none());
      driverXbox.rightBumper().onTrue(Commands.none());
    } else {
      // Teleop controls

      driverXbox.start().onTrue(Commands.runOnce(drivebase::zeroGyro));
      driverXbox.back().whileTrue(Commands.none());
      driverXbox.leftBumper().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
      driverXbox.rightBumper().onTrue(Commands.none());
      // driverXbox.x().onTrue(shooter.reconfigureMotor());
      driverXbox.b().onTrue(shooter.runSystemAtVelocity(SmartDashboard.getNumber("desired rpm: ", 0))).onFalse(shooter.stopSystem());
      // driverXbox.y().onTrue(shooter.runShooterSystem()).onFalse(shooter.stopFullSystem());
      driverXbox.rightBumper().onTrue(shooter.aimAtHub(drivebase));
    }

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.

   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand()
  {
    return autoChooser.getSelected();
  }

  /**
   * Switch between brake or coast mode for the wheels
   * 
   * @param brake True for brake mode
   */
  public void setMotorBrake(boolean brake)
  {
    drivebase.setMotorBrake(brake);
  }
}
