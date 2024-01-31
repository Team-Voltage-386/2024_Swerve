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

    private CANSparkMax aimMotor;
    private CANSparkMax shooterMotor;
    private CANSparkMax rollerMotor;

    private SimpleMotorFeedforward aimFF;
    private SimpleMotorFeedforward ShootFF;
    private SimpleMotorFeedforward RollFF;

    private boolean hasPiece = false;
    // private enum shooterState;

    /**
     * constraints in degrees
     */
    ProfiledPIDController AimPID = new ProfiledPIDController(0, 0, 0, new Constraints(90, 120));
    ProfiledPIDController ShootPID = new ProfiledPIDController(0, 0, 0, new Constraints(10, 10));

    public ShooterSubsystem() {
        // shooterMotor = new CANSparkMax(Shooter.kShooterMotorID, MotorType.kBrushless);
        aimMotor = new CANSparkMax(Shooter.kShooterAimMotorID, MotorType.kBrushless);
        aimMotor.getEncoder().setPosition(Units.degreesToRotations(32));
        aimFF = new SimpleMotorFeedforward(0.0, 0.0);
        // ShootFF = new SimpleMotorFeedforward(0.0, 0.0);
    }

    /**
     * @return Shoot motor RPM
     */
    // public double getShootMotorSpeed() {
    //     return shooterMotor.getEncoder().getVelocity();
    // }

    // public void spoolMotors() {
    //     if(hasPiece) {
    //         shooterMotor.setVoltage(ShootFF.calculate(Shooter.kShooterSpeed) + ShootPID.calculate(getShootMotorSpeed(), Shooter.kShooterSpeed));
    //         rollerMotor.setVoltage(RollFF.calculate(Shooter.kRollerRPM));
    //     }
    // }

    public double getShooterAngle() {
        return Units.rotationsToDegrees(aimMotor.getEncoder().getPosition());
    }

    public void aimShooter(double targetAngle) {
        aimMotor.setVoltage(
            AimPID.calculate(
                getShooterAngle(),
                targetAngle)
            + aimFF.calculate(
                targetAngle-getShooterAngle())
        );
    }

    @Override
    public void periodic() {
        aimShooter(m_aim.getShooterTargetAngle());
    }
}
