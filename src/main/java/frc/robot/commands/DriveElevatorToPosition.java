package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swervedrive.Elevator;

public class DriveElevatorToPosition extends Command{

    private Elevator elevator;
    private double power, position;
    boolean done = false;

    public DriveElevatorToPosition(Elevator elevator, double power, double position){
        this.elevator = elevator;
        this.power = power;
        this.position = position;
        addRequirements(elevator);
    }

    @Override
    public void initialize() {
        //Set direction to run
        if (Elevator.encoder.getPosition() > position) {
            elevator.vertDrive(-power);
        } else {
            elevator.vertDrive(power);
        }
    }

    @Override
    public void execute() {
        //Once it reaches position, hold at the target percent output
        if (Math.abs(Elevator.encoder.getPosition() - position) < 1) {
            elevator.vertDrive(0.025);
        }
    }

    @Override
    public void end(boolean interrupted) {
        done = false;
    }

    @Override
    public boolean isFinished() {
        return done;
    }
    
}
