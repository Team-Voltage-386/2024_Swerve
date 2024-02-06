package frc.robot.Utils;

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

    //PID/FF for chassis rotation speed
    private SimpleMotorFeedforward aimFF = new SimpleMotorFeedforward(0.0, 8);
    private ProfiledPIDController aimPID = new ProfiledPIDController(0.001, 0, 0.0, new Constraints(Math.toRadians(180), Math.toRadians(180)));
    
    private String limelightName = "limelight-a";
    private double limelightHeight = Units.inchesToMeters(24);
    private double targetTagHeight = Units.inchesToMeters(51.88);
    private double speakerHeight = Units.inchesToMeters(82);

    //set default mode
    static boolean speakerMode = true;

    private int targetID = 7;

    public static void toggleMode() {
        speakerMode = !speakerMode;
    }

    //get field relative angle to speaker
    public double getAngleToSpeaker() {
        double toSpeakerAngle = Math.toDegrees(Math.atan((5.55 - m_swerve.getRoboPose2d().getY())/(m_swerve.getRoboPose2d().getX() - 0.3)));
    if(LimelightHelpers.getFiducialID(limelightName) != gamePieceIDs.kSpeakerID)
        return toSpeakerAngle;
    else
        return getLLFRAngleToTarget();
    }

    /**
     * get field relative target angle for swerve to aim at speaker
     * @return radians
     */
    public double getSpeakerAimTargetAngle() { //when geting apriltag data, must invert dir with negative sign to match swerve (try this on tues)
        double Vy = m_swerve.getChassisSpeeds().vyMetersPerSecond + Shooter.kShooterSpeed*Math.sin(Math.toRadians(getAngleToSpeaker()));
        double Vx = m_swerve.getChassisSpeeds().vxMetersPerSecond + Shooter.kShooterSpeed*Math.cos(Math.toRadians(getAngleToSpeaker()));
        System.out.println(2*Math.toRadians(getAngleToSpeaker()) - Math.atan(Vy/Vx));
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
     * robot relative aimbot for non-speaker things
     * @return degrees to the target, right is +, left is -
     */
    public double getLLAngleToTarget() {
        if(hasTarget()) {
        return LimelightHelpers.getTX(limelightName);
        }
        else return 0;
    }

    /**
     * returns field relative angle thats compliant with WPI cause WPI has rotation backwards
     * @return degrees to the target, right is -, left is +
     */
    public double getLLFRAngleToTarget() {
        double angle = m_swerve.getRoboPose2d().getRotation().getDegrees() - LimelightHelpers.getTX(limelightName);
        if(hasTarget())
            return angle;
        else 
            return 0;
    }

    /**
     * calc how fast robot needs to turn to stay with target
     * @return rot speed
     */
    public double getRotationSpeedForTarget() {
        if(getTargetID() != gamePieceIDs.kSpeakerID)
            return -aimFF.calculate(Math.toRadians(getLLAngleToTarget())) - aimPID.calculate(Math.toRadians(getLLAngleToTarget()));
        else
            return hasTarget() 
            ? aimFF.calculate(getSpeakerAimTargetAngle() - m_swerve.getRoboPose2d().getRotation().getRadians())/4 //slow it down cause camera sucks. shouldnt need this with calibrated LL
            : aimFF.calculate(getSpeakerAimTargetAngle() - m_swerve.getRoboPose2d().getRotation().getRadians());// + aimPID.calculate(getRoboPose2d().getRotation().getRadians(), getSpeakerAimTargetAngle());
    }
    
    //pure horizontal distance to tag
    public double getDistToTag() {
        return (targetTagHeight-limelightHeight)/Math.tan(Math.toRadians(LimelightHelpers.getTY(limelightName)));
    }

    //distance to speaker (the hypotenuse, so true distance.)
    public double getDistToSpeaker() {
        return Math.hypot(getDistToTag(), speakerHeight);
    }

    //angle the shooter would need to be at to be pointed directly at the speaker
    public double getVerticalAngleToSpeaker() {
        return Math.atan(speakerHeight/getDistToTag());
    }

    //angle the shooter needs to hit the shot. ask me (Lucas) about the math if you need to cause I dont have time to comment it all rn
    public double getShooterTargetAngle() {
        if(!hasTarget())
            return Shooter.kMinAngle;

        // double Vy = getDistToSpeaker()*((Math.sin(m_shooter.getShooterAngle())/Shooter.kShooterSpeed) no worky
        //  + (Math.sin(getVerticalAngleToSpeaker())/m_swerve.getChassisSpeeds().vyMetersPerSecond));
        // double Vx = getDistToSpeaker()*((Math.cos(m_shooter.getShooterAngle())/Shooter.kShooterSpeed)
        //  + (Math.cos(getVerticalAngleToSpeaker())/m_swerve.getChassisSpeeds().vxMetersPerSecond));

        // double angle = Math.toDegrees(2*getVerticalAngleToSpeaker() - Math.atan(Vy/Vx));
        
        // SmartDashboard.putNumber("Angle before constraints", angle);

        double angle = Math.toDegrees(getVerticalAngleToSpeaker());
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
