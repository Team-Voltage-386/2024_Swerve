package frc.robot.Commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Drivetrain;

public class lockTarget extends Command {
    Drivetrain dt;
    Supplier<Double> xVeloSupplier, yVeloSupplier, rotVeloSupplier;
    Supplier<Boolean> fieldRelativeSupplier, hardLockedSupplier;

    public lockTarget(Drivetrain dt,
            Supplier<Double> xVeloSupplier, Supplier<Double> yVeloSupplier, Supplier<Double> rotVeloSupplier,
            Supplier<Boolean> fieldRelativeSupplier, Supplier<Boolean> hardLockedSupplier) {
        this.dt = dt;
        this.xVeloSupplier = xVeloSupplier;
        this.yVeloSupplier = yVeloSupplier;
        this.rotVeloSupplier = rotVeloSupplier;
        this.fieldRelativeSupplier = fieldRelativeSupplier;
        this.hardLockedSupplier = hardLockedSupplier;
        addRequirements(dt);
    }

    public void schedule() {
        System.out.println("Locking piece.");
    }
    
    @Override
    public void execute() {
        dt.lockPiece(xVeloSupplier.get(), yVeloSupplier.get(), rotVeloSupplier.get(), fieldRelativeSupplier.get(), hardLockedSupplier.get());
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
