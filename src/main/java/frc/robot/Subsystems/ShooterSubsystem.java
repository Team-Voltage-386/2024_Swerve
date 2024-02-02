package frc.robot.Subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ID;
import frc.robot.Utils.Aimlock;

public class ShooterSubsystem extends SubsystemBase {

    private CANSparkMax aimMotor;
    // private CANSparkMax shooterMotor;
    // private CANSparkMax rollerMotor;

    private SimpleMotorFeedforward aimFF;
    // private SimpleMotorFeedforward ShootFF;
    // private SimpleMotorFeedforward RollFF;

    //if this is true it lets the handoff roller motor and the shooter rollers know to start spinning
    private boolean hasPiece = false;

    Aimlock m_aim;
    // private enum shooterState; //dw abt this rn. will use later to specify if AMP or SPEAKER score mode

    /**
     * constraints in degrees
     */
    ProfiledPIDController AimPID = new ProfiledPIDController(0, 0, 0, new Constraints(90, 120));
    ProfiledPIDController ShootPID = new ProfiledPIDController(0, 0, 0, new Constraints(10, 10));

    public ShooterSubsystem() {
        // shooterMotor = new CANSparkMax(ID.kShooterMotorID, MotorType.kBrushless);
        aimMotor = new CANSparkMax(ID.kShooterAimMotorID, MotorType.kBrushless);
        aimMotor.setIdleMode(IdleMode.kBrake);
        aimMotor.getEncoder().setPosition(Units.degreesToRotations(32));
        aimFF = new SimpleMotorFeedforward(0.0, 0.0);
        // ShootFF = new SimpleMotorFeedforward(0.0, 0.0);
    }

    public void setAim(Aimlock m_aim) {
        this.m_aim = m_aim;
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
        SmartDashboard.putNumber("Shooter angle", getShooterAngle());
        SmartDashboard.putNumber("Target angle", m_aim.getShooterTargetAngle());
        SmartDashboard.putNumber("Get vert angle to speaker", m_aim.getVerticalAngleToSpeaker());
        SmartDashboard.putNumber("Get dist to speaker", m_aim.getDistToSpeaker());
        aimShooter(m_aim.getShooterTargetAngle());
    }
}
