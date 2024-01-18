package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Drivetrain;

public class Drive extends Command {
    Drivetrain dt;
    double xVelo, yVelo, rotVelo;
    boolean fieldRelative;

    public Drive(Drivetrain dt, double xVelo, double yVelo, double rotVelo, boolean fieldRelative) {
        this.dt = dt;
        this.xVelo = xVelo;
        this.yVelo = yVelo;
        this.rotVelo = rotVelo;
        this.fieldRelative = fieldRelative;
    }

    public void schedule() {
        System.out.println("Driving.");
    }
    
    @Override
    public void execute() {
        dt.drive(xVelo, yVelo, rotVelo, fieldRelative);
        
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
