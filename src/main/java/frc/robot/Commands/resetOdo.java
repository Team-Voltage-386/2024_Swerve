package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Drivetrain;

public class resetOdo extends Command {
    Drivetrain dt;

    public resetOdo(Drivetrain dt) {
        this.dt = dt;
    }

    @Override
    public void schedule() {
        System.out.println("resetting odometry");
        dt.resetGyro();
        dt.resetOdo();
        dt.resetOdo(); //yes, call it twice.
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
