package frc.robot.Utils;

import frc.robot.RobotContainer;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.Shooter;
import frc.robot.Constants.gamePieceIDs;
import frc.robot.Subsystems.Drivetrain;
import frc.robot.Subsystems.ShooterSubsystem;

public class Aimlock {
    Drivetrain m_swerve;
    ShooterSubsystem m_shooter;

    public Aimlock (Drivetrain m_swerve, ShooterSubsystem m_shooter) {
        this.m_swerve = m_swerve;
        this.m_shooter = m_shooter;
    }

    private SimpleMotorFeedforward aimFF = new SimpleMotorFeedforward(0.0, 8);
    private ProfiledPIDController aimPID = new ProfiledPIDController(0.05, 0, 0.0, new Constraints(Math.toRadians(180), Math.toRadians(180)));
    
    private String limelightName = "limelight-a";
    private double limelightHeight = Units.inchesToMeters(24);
    private double targetTagHeight = Units.inchesToMeters(51.88);
    private double speakerHeight = Units.inchesToMeters(82);

    static boolean speakerMode = true;

    private int targetID = 7;

    public static void toggleMode() {
        speakerMode = !speakerMode;
    }


    public double getAngleToSpeaker() {
        double toSpeakerAngle = Math.toDegrees(Math.atan((5.55 - m_swerve.getRoboPose2d().getY())/(m_swerve.getRoboPose2d().getX() - 0.3)));
    if(LimelightHelpers.getFiducialID(limelightName) != gamePieceIDs.kSpeakerID)
        return toSpeakerAngle;
    else
        return getLLFRAngleToTarget();
    }

    /**
     * @return radians
     */
    public double getSpeakerAimTargetAngle() { //when geting apriltag data, must invert dir with negative sign to match swerve (try this on tues)
        double Vy = m_swerve.getChassisSpeeds().vyMetersPerSecond + Shooter.kShooterSpeed*Math.sin(Math.toRadians(getAngleToSpeaker()));
        double Vx = m_swerve.getChassisSpeeds().vxMetersPerSecond + Shooter.kShooterSpeed*Math.cos(Math.toRadians(getAngleToSpeaker()));
        return 2*Math.toRadians(getAngleToSpeaker()) - Math.atan(Vy/Vx);
    }

    public void setTarget(int targetID) {
        this.targetID = targetID;
    }

    public int getTargetID() {
        return this.targetID;
    }

    public boolean hasTarget() {
        return LimelightHelpers.getFiducialID(limelightName) == getTargetID();
    }

    /**
     * @return degrees to the target, right is +, left is -
     */
    public double getLLAngleToTarget() {
        if(hasTarget()) {
        return LimelightHelpers.getTX(limelightName);
        }
        else return 0;
    }

    /**
     * @return degrees to the target, right is -, left is +
     */
    public double getLLFRAngleToTarget() {
        double angle = m_swerve.getRoboPose2d().getRotation().getDegrees() - LimelightHelpers.getTX(limelightName);
        if(hasTarget())
            return angle;
        else 
            return 0;
    }

    public double getRotationSpeedForTarget() {
        if(getTargetID() != gamePieceIDs.kSpeakerID)
            return -aimFF.calculate(Math.toRadians(getLLAngleToTarget())) - aimPID.calculate(Math.toRadians(getLLAngleToTarget()));
        else
            return hasTarget() 
            ? aimFF.calculate(getSpeakerAimTargetAngle() - m_swerve.getRoboPose2d().getRotation().getRadians())/4 //slow it down cause camera sucks. shouldnt need this with calibrated LL
            : aimFF.calculate(getSpeakerAimTargetAngle() - m_swerve.getRoboPose2d().getRotation().getRadians());// + aimPID.calculate(getRoboPose2d().getRotation().getRadians(), getSpeakerAimTargetAngle());
    }

    public double getDistToTag() {
        return (targetTagHeight-limelightHeight)*(1/Math.tan(LimelightHelpers.getTY("")));
    }

    public double getDistToSpeaker() {
        return Math.hypot(getDistToTag(), speakerHeight);
    }

    public double getVerticalAngleToSpeaker() {
        return Math.atan(speakerHeight/getDistToTag());
    }

    public double getShooterTargetAngle() {
        if(!hasTarget())
            return Shooter.kMinAngle;

        double Vy = getDistToSpeaker()*((Math.sin(m_shooter.getShooterAngle())/Shooter.kShooterSpeed)
         + (Math.sin(getVerticalAngleToSpeaker())/m_swerve.getChassisSpeeds().vyMetersPerSecond));
        double Vx = getDistToSpeaker()*((Math.cos(m_shooter.getShooterAngle())/Shooter.kShooterSpeed)
         + (Math.cos(getVerticalAngleToSpeaker())/m_swerve.getChassisSpeeds().vxMetersPerSecond));
        double angle = 2*getVerticalAngleToSpeaker() - Math.atan(Vy/Vx);
        SmartDashboard.putNumber("Angle before constraints", angle);
        if(angle >= Shooter.kMinAngle && angle <= Shooter.kMaxAngle)
            return angle;
        else {
            if(angle > Shooter.kMaxAngle)
                return Shooter.kMaxAngle;
            else
                return Shooter.kMinAngle;
        }
    }

}
