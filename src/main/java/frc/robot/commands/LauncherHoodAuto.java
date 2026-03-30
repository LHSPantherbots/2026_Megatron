package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Launcher;
import frc.robot.subsystems.Leds;
import frc.robot.util.LedStatus;
import frc.robot.subsystems.Hood;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class LauncherHoodAuto extends Command {
    Launcher launcher;
    Hood hood;
    Leds leds;
    CommandSwerveDrivetrain drivetrain;
    boolean shouldEnd = false;
    LedStatus ledStatus = LedStatus.FRONT_LOCK;
    
    public LauncherHoodAuto(
            Launcher launcher, Hood hood, Leds leds, CommandSwerveDrivetrain drivetrain) {
        this.launcher = launcher;
        this.hood = hood;
        this.leds = leds;
        this.drivetrain = drivetrain;
        addRequirements(launcher, hood, leds
        );
    }
    
    @Override
    public void initialize() {
        calcHoodAndLauncherSetpoint();
    }

    @Override
    public void execute() {
        calcHoodAndLauncherSetpoint();
        this.hood.closedLoopHood();
        this.launcher.closedLoopVelocityLaunchVoltage();
        
        if(this.launcher.isAtSpeed() && this.hood.isAtAngle()){
            setLedsFin(ledStatus);
        }else{
            setLedsExe(ledStatus);
        }

        
    }

    

    @Override
    public void end(boolean interrupted) {
      this.hood.setHoodShort();
      this.launcher.stopLauncher();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    public void calcHoodAndLauncherSetpoint(){

    double[] values = drivetrain.getLengthAndAngleFromHub();
        double distance = values[0];
        SmartDashboard.putNumber("Distance", distance);

      if (distance<3){

        
        double hoodAngle = 0.02*(distance) + 0.77;
        hood.setHoodMode(hoodAngle);

        double launcherSpeed = 5*(distance) + (45.0 + 3.0);
        launcher.setLauncherMode(launcherSpeed);
        }
        else{
            double hoodAngle = 0.0045*(distance) + 0.8164;
        hood.setHoodMode(hoodAngle);

        double launcherSpeed = 6.18*(distance) + (39.5 + 3.0);
        launcher.setLauncherMode(launcherSpeed);    
        }
        
    }


    public void setLedsExe(LedStatus status){
        switch (status) {
      case FRONT_LOCK:
        leds.purple();
        break;
      case REAR_LOCK:
        leds.yellow();
        break;
      case NONE:
        leds.orange();
        break;
      
    }
    }

    public void setLedsFin(LedStatus status){
        switch (status) {
      case FRONT_LOCK:
        leds.purpleFlash();;
        break;
      case REAR_LOCK:
        leds.yellowFlash();;
        break;
      case NONE:
        leds.orangeFlash();;
        break;
      
    }
    }


 }

