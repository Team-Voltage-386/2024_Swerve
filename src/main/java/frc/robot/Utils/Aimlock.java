package frc.robot.Utils;

import frc.robot.RobotContainer;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.DriveTrain;
import frc.robot.Constants.Shooter;
import frc.robot.Constants.gamePieceIDs;
import frc.robot.Subsystems.Drivetrain;
import frc.robot.Subsystems.ShooterSubsystem;

public class Aimlock {
    RobotContainer m_container = new RobotContainer();
    Drivetrain m_swerve = m_container.getDrivetrain();
    ShooterSubsystem m_shooter = m_container.getShooter();

    private SimpleMotorFeedforward aimFF = new SimpleMotorFeedforward(0.0, 8);
    private ProfiledPIDController aimPID = new ProfiledPIDController(0.05, 0, 0.0, new Constraints(Math.toRadians(180), Math.toRadians(180)));

    String limelightName = "limelight-a";
    private double limelightHeight = Units.inchesToMeters(24);
    private double targetTagHeight = Units.inchesToMeters(51.88);
    private double speakerHeight = Units.inchesToMeters(82);

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
        double Vy = m_swerve.getChassisSpeeds().vyMetersPerSecond + 10*Math.sin(Math.toRadians(getAngleToSpeaker()));
        double Vx = m_swerve.getChassisSpeeds().vxMetersPerSecond + 10*Math.cos(Math.toRadians(getAngleToSpeaker()));
        return 2*Math.toRadians(getAngleToSpeaker()) - Math.atan(Vy/Vx);
    }

    private int targetID = 1;

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
        double Vy = getDistToSpeaker()*((Math.sin(m_shooter.getShooterAngle())/Shooter.kShooterSpeed)
         + (Math.sin(getVerticalAngleToSpeaker())/m_swerve.getChassisSpeeds().vyMetersPerSecond));
        double Vx = getDistToSpeaker()*((Math.cos(m_shooter.getShooterAngle())/Shooter.kShooterSpeed)
         + (Math.cos(getVerticalAngleToSpeaker())/m_swerve.getChassisSpeeds().vxMetersPerSecond));
        double angle = 2*getVerticalAngleToSpeaker() - Math.atan(Vy/Vx);
        if(angle >= 32 && angle <= 52)
            return angle;
        else {
            if(angle > 52)
                return 52;
            else
                return 32;
        }
    }

}
