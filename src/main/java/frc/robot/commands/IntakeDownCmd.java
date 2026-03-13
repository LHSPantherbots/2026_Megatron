package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakePivot;

public class IntakeDownCmd extends Command {
    IntakePivot intakePivot;
    boolean finishes = false;
    

    public IntakeDownCmd( IntakePivot intakePivot, boolean finishes) {
        this.intakePivot = intakePivot;
        this.finishes = finishes;
        addRequirements(intakePivot);
    }

    @Override
    public void initialize() {
        this.intakePivot.setIntakeDown();
        this.intakePivot.setQuickPID();
    }

    @Override
    public void execute() {
        this.intakePivot.motionMagicSetPosition();
    }

    @Override
    public void end(boolean interrupted) {
        this.intakePivot.motionMagicSetPosition();
        this.intakePivot.setSoftPID();
    }

    @Override
    public boolean isFinished() {
        if (this.finishes) {
        return this.intakePivot.isAtPosition();
        } else {
        return false;
        }
    }

}