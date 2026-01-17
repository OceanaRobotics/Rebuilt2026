package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.EndEffector;

public class RunEndEffector extends Command{
    private EndEffector endEffector;
    private boolean done = false;

    public RunEndEffector(EndEffector endEffector){
        this.endEffector = endEffector;
        addRequirements(endEffector);
    }

    @Override
    public void initialize() {
        // Just run at percent power
        endEffector.intake(0.75);
    }


    @Override
    public void end(boolean interrupted) {
        endEffector.intake(0);
    }

    @Override
    public boolean isFinished() {
        return done;
    }
}
