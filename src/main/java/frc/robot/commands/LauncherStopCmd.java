
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Launcher;
import frc.robot.subsystems.Leds;

public class LauncherStopCmd extends Command {
    Launcher launcher;
    Hood hood;
    Leds leds;
    boolean finishes;

    public LauncherStopCmd(Launcher launcher, Hood hood, Leds leds, boolean finishes) {
        this.launcher = launcher;
        this.hood = hood;
        this.leds = leds;
        this.finishes = finishes;
        addRequirements(launcher, hood, leds);
    }

    @Override
    public void initialize() {
        this.launcher.stopLauncher();
        this.hood.setHoodShort();
    }

    @Override
    public void execute() {
        this.launcher.closedLoopVelocityLaunchVoltage();
        this.hood.closedLoopHood();
        this.leds.rainbow();
    }

    @Override
    public void end(boolean interrupted) {
        this.launcher.closedLoopVelocityLaunchVoltage();
        this.hood.closedLoopHood();
        this.leds.rainbow();
    }

    @Override
    public boolean isFinished() {
        if (this.finishes) {
        return (this.hood.isAtAngle());
        } else {
        return false;
        }
    }


}
