package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakePivot;

public class IntakeDownCmd extends Command {
    IntakePivot intakePivot;
    boolean shouldEnd = false;
    

    public IntakeDownCmd( IntakePivot intakePivot) {
        this.intakePivot = intakePivot;
        addRequirements(intakePivot);
    }

    @Override
    public void initialize() {
      this.intakePivot.setIntakeDown();
    }

    @Override
    public void execute() {
    this.intakePivot.motionMagicSetPosition();
    }

    @Override
    public void end(boolean interrupted) {
        this.shouldEnd = false;
    }

    @Override
    public boolean isFinished() {
        return this.shouldEnd;
    }

}