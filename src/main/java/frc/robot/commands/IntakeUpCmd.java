package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakePivot;

public class IntakeUpCmd extends Command {
    IntakePivot intakePivot;
    boolean finishes = false;
    

    public IntakeUpCmd( IntakePivot intakePivot, boolean finishes) {
        this.intakePivot = intakePivot;
        this.finishes = finishes;
        addRequirements(intakePivot);
    }

    @Override
    public void initialize() {
        this.intakePivot.setIntakeUp();
        this.intakePivot.setQuickPID();
    }

    @Override
    public void execute() {
        this.intakePivot.motionMagicSetPosition();
    }

    @Override
    public void end(boolean interrupted) {
        this.intakePivot.motionMagicSetPosition();
        this.intakePivot.setQuickPID();
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