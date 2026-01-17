package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.EndEffector;

public class EndEffectorSequencing extends Command{
    private EndEffector endEffector;
    private boolean beamBroke, beamRestored = false;

    public EndEffectorSequencing(EndEffector endEffector){
        this.endEffector = endEffector;
    }

    @Override
    public void initialize() {
        //Initial percent output to run motor at
        endEffector.intake(0.50); //faster speed
        beamBroke = false;
        beamRestored = false;
    }

    @Override
    public void execute() {
        //Once LaserCan sees coral, slow down the motor speed
        if(EndEffector.laserCanDist() < 10){
            endEffector.intake(0.25); //slower speed
            beamBroke = true;
        }

        //After coral passes through the LaserCan, stop motor
        if(EndEffector.laserCanDist() > 130 && beamBroke == true){
            endEffector.intake(0.0);
            beamRestored = true;
        }
    }

    @Override
    public void end(boolean interrupted) {
        //Reset variables to reuse command
        beamBroke = false;
        beamRestored = false;
    }

    @Override
    public boolean isFinished() {
        //Stop Command once passed the LaserCan
        return beamRestored;
    }
}
