package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Launcher;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class LauncherHoodAutoPersist extends Command {
    Launcher launcher;
    Hood hood;
    CommandSwerveDrivetrain drivetrain;
    boolean shouldEnd = false;
    
    public LauncherHoodAutoPersist(
            Launcher launcher, Hood hood, CommandSwerveDrivetrain drivetrain) {
        this.launcher = launcher;
        this.hood = hood;
        this.drivetrain = drivetrain;
        addRequirements(launcher, hood);
    }
    
    @Override
    public void initialize() {
        double[] values = drivetrain.getLengthAndAngleFromHub();
        double distance = values[0];
        SmartDashboard.putNumber("Distance", distance);
        if (distance<3){

        
        double hoodAngle = 0.02*(distance) + 0.77;
        hood.setHoodMode(hoodAngle);

        double launcherSpeed = 5*(distance) + (45.0 + 5.0);
        launcher.setLauncherMode(launcherSpeed);
        }
        else{
            double hoodAngle = 0.0045*(distance) + 0.8164;
        hood.setHoodMode(hoodAngle);

        double launcherSpeed = 6.18*(distance) + (39.5 + 5.0);
        launcher.setLauncherMode(launcherSpeed);    
        }
    }

    @Override
    public void execute() {
        this.hood.closedLoopHood();
        this.launcher.closedLoopVelocityLaunchVoltage();
        
    }

    @Override
    public void end(boolean interrupted) {
        this.hood.closedLoopHood();
        this.launcher.closedLoopVelocityLaunchVoltage();
    }

    @Override
    public boolean isFinished() {
    return (this.launcher.isAtSpeed() && this.hood.isAtAngle());
    }
 }

