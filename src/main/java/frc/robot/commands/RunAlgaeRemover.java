package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeRemover;


public class RunAlgaeRemover extends Command {
    
 private AlgaeRemover algaeRemover;
    private double algaePower, algaeTargetPosition;
    boolean done = false;

    public RunAlgaeRemover(AlgaeRemover algaeRemover, double algaePower, double algaeTargetPosition){
        this.algaeRemover = algaeRemover;
        this.algaePower = algaePower;
        this.algaeTargetPosition = algaeTargetPosition;
        addRequirements(algaeRemover);
    }

    @Override
    public void initialize() {
        //Set direction to run
        if (AlgaeRemover.algaeEncoder.getPosition() > algaeTargetPosition) {
            algaeRemover.AlgaeMotorSpeed(-algaePower);
        } else {
            algaeRemover.AlgaeMotorSpeed(algaePower);
        }
    }

    @Override
    public void execute() {
        if (Math.abs(AlgaeRemover.algaeEncoder.getPosition() - algaeTargetPosition) < 1){
                algaeRemover.AlgaeMotorSpeed(0.05); // TROUBLE CODE
        }
        // else {
        //     algaeRemover.AlgaeMotorSpeed();
        // }

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