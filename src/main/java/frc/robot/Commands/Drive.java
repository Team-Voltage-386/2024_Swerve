package frc.robot.Commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Drivetrain;

public class Drive extends Command {
    Drivetrain dt;
    Supplier<Double> xVeloSupplier, yVeloSupplier, rotVeloSupplier;
    Supplier<Boolean> fieldRelativeSupplier;

    public Drive(Drivetrain dt,
            Supplier<Double> xVeloSupplier, Supplier<Double> yVeloSupplier, Supplier<Double> rotVeloSupplier,
            Supplier<Boolean> fieldRelativeSupplier) {
        this.dt = dt;
        this.xVeloSupplier = xVeloSupplier;
        this.yVeloSupplier = yVeloSupplier;
        this.rotVeloSupplier = rotVeloSupplier;
        this.fieldRelativeSupplier = fieldRelativeSupplier;
        addRequirements(dt);
    }

    public void schedule() {
        System.out.println("Driving.");
    }
    
    @Override
    public void execute() {
        dt.drive(xVeloSupplier.get(), yVeloSupplier.get(), rotVeloSupplier.get(), fieldRelativeSupplier.get());
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
