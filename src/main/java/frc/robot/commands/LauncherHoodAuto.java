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
    LedStatus ledStatus = LedStatus.NONE;
    
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
        
        if (LimelightHelpers.getTV("limelight-front")) {
            drivetrain.setposefromlimelightFront();
            ledStatus = LedStatus.FRONT_LOCK;  
        } else if(LimelightHelpers.getTV("limelight-rr")){
            drivetrain.setposefromlimelight();
            ledStatus = LedStatus.REAR_LOCK;
        }else{
            ledStatus = LedStatus.NONE;
        }


        double[] values = drivetrain.getLengthAndAngleFromHub();
        double distance = values[0];
        SmartDashboard.putNumber("Distance", distance);

        double hoodAngle = MathUtil.clamp(0.02*(distance) + 0.77, hood.getHoodMin(), hood.getHoodMax());
        hood.setHoodMode(hoodAngle);

        double launcherSpeed = 5*(distance) + (45.0+5.0);
        launcher.setLauncherMode(launcherSpeed);
    }

    @Override
    public void execute() {
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

