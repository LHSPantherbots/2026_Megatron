// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeRoller;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class IntakeCmd extends Command {
  private final IntakeRoller intakeRoller;
  /** Creates a new Intake. */
  public IntakeCmd(IntakeRoller intakeRoller) {
    this.intakeRoller = intakeRoller;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intakeRoller);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intakeRoller.intake();
    System.out.println("Intaking");
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intakeRoller.stop();
    System.out.println("Stopped Intaking");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
