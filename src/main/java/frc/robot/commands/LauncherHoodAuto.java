package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Launcher;
import frc.robot.subsystems.Hood;
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

       if(values[0] < 1) {
        hood.setHoodShort();
        launcher.setLauncherShort();
       } else if (values[0] < 2) {
        hood.setHoodMid();
        launcher.setLauncherMid();
       } else if (values[0] < 3) {
        hood.setHoodLong();
        launcher.setLauncherLong();
       } else {
        hood.setHoodExtraLong();
        launcher.setLauncherExtraLong();
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

