package frc.robot.Subsystems;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Shooter;
import frc.robot.Utils.LimelightHelpers;
import frc.robot.Utils.Aimlock;

public class ShooterSubsystem extends SubsystemBase {
    private Aimlock m_aim = new Aimlock();

    private CANSparkMax AimMotor;
    // private CANSparkMax shooterMotor;

    private SimpleMotorFeedforward AimFF;
    // private SimpleMotorFeedforward ShootFF;

    /**
     * constraints in degrees
     */
    ProfiledPIDController AimPID = new ProfiledPIDController(0, 0, 0, new Constraints(90, 120));
    ProfiledPIDController ShootPID = new ProfiledPIDController(0, 0, 0, new Constraints(10, 10));

    public ShooterSubsystem() {
        // shooterMotor = new CANSparkMax(Shooter.kShooterMotorID, MotorType.kBrushless);
        AimMotor = new CANSparkMax(Shooter.kShooterAimMotorID, MotorType.kBrushless);
        AimMotor.getEncoder().setPosition(Units.degreesToRotations(30));
        AimFF = new SimpleMotorFeedforward(0.0, 0.0);
        // ShootFF = new SimpleMotorFeedforward(0.0, 0.0);
    }

    // /**
    //  * @return Shoot motor RPM
    //  */
    // public double getShootMotorSpeed() {
    //     return shooterMotor.getEncoder().getVelocity();
    // }

    // public void spoolMotors() {
    //     shooterMotor.setVoltage(ShootFF.calculate(Shooter.kShooterSpeed) + ShootPID.calculate(getShootMotorSpeed(), Shooter.kShooterSpeed));
    // }

    public double getShooterAngle() {
        return Units.rotationsToDegrees(AimMotor.getEncoder().getPosition());
    }

    public void aimShooter(double targetAngle) {
        AimMotor.setVoltage(
            AimPID.calculate(
                getShooterAngle(),
                targetAngle)
            + AimFF.calculate(
                targetAngle-getShooterAngle())
        );
    }

    @Override
    public void periodic() {
        aimShooter(m_aim.getShooterTargetAngle());
    }
}
