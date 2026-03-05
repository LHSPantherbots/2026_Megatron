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
        addRequirements(launcher, hood, drivetrain);
    }
    
    @Override
    public void initialize() {
        double[] values = drivetrain.getLengthAndAngleFromHub();
        double distance = values[1];

        double hoodangle = 0.02*(distance) + 0.77;
        hood.setLauncherMode(hoodangle);

         if(values[0] < 1) {
        launcher.setLauncherMode(Positions.SHORT);
       } else if (values[0] < 2) {
        launcher.setLauncherMode(Positions.MID);
       } else if (values[0] < 3) {
        launcher.setLauncherMode(Positions.LONG);
       } else {
        launcher.setLauncherMode(Positions.EXTRALONG);
       }
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

