package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Launcher;
import frc.robot.subsystems.Hood;
import frc.robot.Positions;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class LauncherHoodAuto extends Command {
    Launcher launcher;
    Hood hood;
    CommandSwerveDrivetrain drivetrain;
    boolean noteWasDetected = false;
    boolean shouldEnd = false;
    
    public LauncherHoodAuto(
            Launcher launcher, Hood hood, CommandSwerveDrivetrain drivetrain) {
        this.launcher = launcher;
        this.hood = hood;
        this.drivetrain = drivetrain;
        addRequirements(launcher, hood);
    }
    
    @Override
    public void initialize() {
        double[] values = drivetrain.getLengthAndAngleFromHub();
        double distance = values[1];

        double hoodAngle = 0.02*(distance) + 0.77;
        hood.setHoodMode(hoodAngle);

        double launcherSpeed = 5*(distance) + 45;
        launcher.setLauncherMode(launcherSpeed);
    }

    @Override
    public void execute() {

    }

    @Override
    public void end(boolean interrupted) {
      
    }

    @Override
    public boolean isFinished() {
        return true;
    }
 }

