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
import frc.robot.Subsystems.Drivetrain;

public class ShooterSubsystem extends SubsystemBase {
    private double limelightHeight = Units.inchesToMeters(24);
    private double targetTagHeight = Units.inchesToMeters(51.88);
    private double speakerHeight = Units.inchesToMeters(82);
    private double motorSpeed;

    private CANSparkMax AimMotor;
    private CANSparkMax shooterMotor;

    private SimpleMotorFeedforward AimFF;
    private SimpleMotorFeedforward ShootFF;

    /**
     * constraints in degrees
     */
    ProfiledPIDController AimPID = new ProfiledPIDController(0, 0, 0, new Constraints(90, 120));
    ProfiledPIDController ShootPID = new ProfiledPIDController(0, 0, 0, new Constraints(10, 10));

    private Drivetrain m_swerve;

    public ShooterSubsystem(Drivetrain m_swerve) {
        this.m_swerve = m_swerve;
        shooterMotor = new CANSparkMax(Shooter.kShooterMotorID, MotorType.kBrushless);
        AimMotor = new CANSparkMax(Shooter.kShooterAimMotorID, MotorType.kBrushless);
        AimMotor.getEncoder().setPosition(Units.degreesToRotations(30));
        AimFF = new SimpleMotorFeedforward(0.0, 0.0);
        ShootFF = new SimpleMotorFeedforward(0.0, 0.0);
        motorSpeed = 10;
    }

    /**
     * @return Shoot motor RPM
     */
    public double getShootMotorSpeed() {
        return shooterMotor.getEncoder().getVelocity();
    }

    public void spoolMotors() {
        shooterMotor.setVoltage(ShootFF.calculate(motorSpeed) + ShootPID.calculate(getShootMotorSpeed(), motorSpeed));
    }

    public double getDistToTag() {
        return (targetTagHeight-limelightHeight)*(1/Math.tan(LimelightHelpers.getTY("")));
    }

    public double getDistToSpeaker() {
        return Math.hypot(getDistToTag(), speakerHeight);
    }

    public double getAngleToSpeaker() {
        return Math.atan(speakerHeight/getDistToTag());
    }

    public double getShooterAngle() {
        return Units.rotationsToDegrees(AimMotor.getEncoder().getPosition());
    }

    public double getShooterTargetAngle() {
        double Vy = getDistToSpeaker()*((Math.sin(getShooterAngle())/motorSpeed)
         + (Math.sin(m_swerve.getAngleToSpeaker())/m_swerve.getChassisSpeeds().vyMetersPerSecond));
        double Vx = getDistToSpeaker()*((Math.cos(getShooterAngle())/motorSpeed)
         + (Math.cos(m_swerve.getAngleToSpeaker())/m_swerve.getChassisSpeeds().vxMetersPerSecond));
        double angle = 2*getAngleToSpeaker() - Math.atan(Vy/Vx);
        if(angle >= 32 && angle <= 52)
            return angle;
        else {
            if(angle > 52)
                return 52;
            else
                return 32;
        }
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
        aimShooter(getShooterTargetAngle());
    }
}
