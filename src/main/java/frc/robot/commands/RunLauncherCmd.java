
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Launcher;
import frc.robot.subsystems.Leds;
public class RunLauncherCmd extends Command {
    Launcher launcher;
    Leds leds;
    boolean finishes;


    public RunLauncherCmd(Launcher launcher, Leds leds, boolean finishes) {
        this.launcher = launcher;
        this.leds = leds;
        this.finishes = finishes;
        addRequirements(launcher, leds);
    }

    @Override
    public void initialize() {
        this.launcher.setLauncherMid();
        
    }

    @Override
    public void execute() {
        this.launcher.closedLoopVelocityLaunchVoltage();
        this.leds.blue();
    }

    @Override
    public void end(boolean interrupted) {
        this.launcher.closedLoopVelocityLaunchVoltage();
        this.leds.blue();
    }

    @Override
    public boolean isFinished() {
        if (this.finishes) {
        return (this.launcher.isAtSpeed());
        } else {
        return false;
        }
    }


}
