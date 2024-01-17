// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Optional;

import com.ctre.phoenix.sensors.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants.ID;
import frc.robot.Constants.Offsets;
import frc.robot.Utils.LimelightHelpers;
import frc.robot.Constants.DriveTrain;

/** Represents a swerve drive style drivetrain. */
public class Drivetrain implements Subsystem {
    public static final double kMaxPossibleSpeed = 4.9; // meters per second
    public static final double kMaxAngularSpeed = 3 * Math.PI; // per second

    private final Translation2d m_frontLeftLocation = new Translation2d(
            DriveTrain.kDistanceMiddleToFrontMotor * DriveTrain.kXForward,
            DriveTrain.kDistanceMiddleToSideMotor * DriveTrain.kYLeft);
    private final Translation2d m_frontRightLocation = new Translation2d(
            DriveTrain.kDistanceMiddleToFrontMotor * DriveTrain.kXForward,
            DriveTrain.kDistanceMiddleToSideMotor * DriveTrain.kYRight);
    private final Translation2d m_backLeftLocation = new Translation2d(
            DriveTrain.kDistanceMiddleToFrontMotor * DriveTrain.kXBackward,
            DriveTrain.kDistanceMiddleToSideMotor * DriveTrain.kYLeft);
    private final Translation2d m_backRightLocation = new Translation2d(
            DriveTrain.kDistanceMiddleToFrontMotor * DriveTrain.kXBackward,
            DriveTrain.kDistanceMiddleToSideMotor * DriveTrain.kYRight);

    private final SwerveModule m_frontLeft = new SwerveModule("FrontLeft",
            ID.kFrontLeftDrive,
            ID.kFrontLeftTurn,
            ID.kFrontLeftCANCoder,
            Offsets.kFrontLeftOffset,
            DriveTrain.turnPID,
            DriveTrain.drivePID,
            DriveTrain.turnFeedForward,
            DriveTrain.driveFeedForward);
    private final SwerveModule m_frontRight = new SwerveModule("FrontRight",
            ID.kFrontRightDrive,
            ID.kFrontRightTurn,
            ID.kFrontRightCANCoder,
            Offsets.kFrontRightOffset,
            DriveTrain.turnPID,
            DriveTrain.drivePID,
            DriveTrain.turnFeedForward,
            DriveTrain.driveFeedForward);
    private final SwerveModule m_backLeft = new SwerveModule("BackLeft",
            ID.kBackLeftDrive,
            ID.kBackLeftTurn,
            ID.kBackLeftCANCoder,
            Offsets.kBackLeftOffset,
            DriveTrain.turnPID,
            DriveTrain.drivePID,
            DriveTrain.turnFeedForward,
            DriveTrain.driveFeedForward);
    private final SwerveModule m_backRight = new SwerveModule("BackRight",
            ID.kBackRightDrive,
            ID.kBackRightTurn,
            ID.kBackRightCANCoder,
            Offsets.kBackRightOffset,
            DriveTrain.turnPID,
            DriveTrain.drivePID,
            DriveTrain.turnFeedForward,
            DriveTrain.driveFeedForward);

    private final Pigeon2 m_gyro = new Pigeon2(ID.kGyro);

    /**
     * The order that you initialize these is important! Later uses of functions
     * like toSwerveModuleStates will return the same order that these are provided.
     * See
     * https://docs.wpilib.org/en/stable/docs/software/kinematics-and-odometry/swerve-drive-kinematics.html
     */
    private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
            m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);

    private final SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(
            m_kinematics,
            getGyroYawRotation2d(),
            new SwerveModulePosition[] {
                    m_frontLeft.getPosition(),
                    m_frontRight.getPosition(),
                    m_backLeft.getPosition(),
                    m_backRight.getPosition()
            });

    private ChassisSpeeds m_chassisSpeeds = new ChassisSpeeds();
    private Pose2d robotFieldPosition = getRoboPose2d();

    public Drivetrain() {
        // Zero at beginning of match. Zero = whatever direction the robot (more specifically the gyro) is facing
        //m_gyro.
        this.resetGyro();

        // Configure AutoBuilder last
        AutoBuilder.configureHolonomic(
                this::getRoboPose2d, // Robot pose supplier
                this::resetOdo, // Method to reset odometry (will be called if your auto has a starting pose)
                this::getChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                this::driveWithChasisSpeeds, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
                new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                        new PIDConstants(7, 0.0, 0.0), // Translation PID constants
                        new PIDConstants(5.4, 0.0, 0.0), // Rotation PID constants
                        kMaxPossibleSpeed, // Max module speed, in m/s
                        DriveTrain.kDriveBaseRadius, // Drive base radius in meters. Distance from robot center to furthest module.
                        new ReplanningConfig() // Default path replanning config. See the API for the options here
                ),
                () -> {
                    // Boolean supplier that controls when the path will be mirrored for the red alliance
                    // This will flip the path being followed to the red side of the field.
                    // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                },
                this // Reference to this subsystem to set requirements
        );
        PPHolonomicDriveController.setRotationTargetOverride(this::getRotationTargetOverride);
    }



    public Optional<Rotation2d> getRotationTargetOverride(){
        // Some condition that should decide if we want to override rotation
        if(hasTarget()) {
                // Return an optional containing the rotation override (this should be a field relative rotation)
                return Optional.of(Rotation2d.fromDegrees(-getRobotRotToTarg())); //todo try a negative target.
        } else {
                // return an empty optional when we don't want to override the path's rotation
                return Optional.empty();
        }
    }

    /**in degrees */
    public double getRobotRotToTarg() {
        double targRot;
        if(hasTarget())
        return MathUtil.inputModulus(-m_gyro.getYaw() + LimelightHelpers.getTX(""), -180, 180);
        else
        return 0;
    }

    private double targetID; //conv to string.

    public void setTarget(double targetID) {
        this.targetID = targetID;
    }

    public double getTarget() {
        return this.targetID;
    }

    public boolean hasTarget() {
        return LimelightHelpers.getFiducialID("") == getTarget();
    }

    public void resetGyro() {
        m_gyro.setYaw(0);
    }

    public void resetOdo() {
        m_odometry.resetPosition(getGyroYawRotation2d(), getModulePositions(), robotFieldPosition);
    }

    public void resetOdo(Pose2d pose) {
        //System.out.println(getGyroYawRotation2d().getDegrees());
        //System.out.println(pose.getX() + " " + pose.getY() + " " + pose.getRotation().getDegrees() + " %%%%%%%%%%%%%%$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$%%%%");
        m_odometry.resetPosition(getGyroYawRotation2d(), getModulePositions(), pose);
    }

    public ChassisSpeeds getChassisSpeeds() {
        return m_chassisSpeeds;
    }

    public SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[] {
                m_frontLeft.getPosition(),
                m_frontRight.getPosition(),
                m_backLeft.getPosition(),
                m_backRight.getPosition()
        };
    }

    /**
     * Sends swerve info to smart dashboard
     */
    public void print() {
        m_frontLeft.print();
        m_frontRight.print();
        m_backLeft.print();
        m_backRight.print();
    }

    /**
     * Get the yaw of gyro in Rotation2d form
     * 
     * @return chasis angle in Rotation2d
     */
    public Rotation2d getGyroYawRotation2d() {
        return Rotation2d.fromDegrees(MathUtil.inputModulus(-m_gyro.getYaw(), -180, 180));
    }

    /**
     * Method to drive the robot using joystick info.
     *
     * @param xSpeed        Speed of the robot in the x direction (forward).
     * @param ySpeed        Speed of the robot in the y direction (sideways).
     * @param rotSpeed      Angular rate of the robot.
     * @param fieldRelative Whether the provided x and y speeds are relative to the
     *                      field.
     */
    public void drive(double xSpeed, double ySpeed, double rotSpeed, boolean fieldRelative) {
        SwerveModuleState[] swerveModuleStates = m_kinematics.toSwerveModuleStates(
                fieldRelative
                        ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rotSpeed,
                                getGyroYawRotation2d())
                        : new ChassisSpeeds(xSpeed, ySpeed, rotSpeed));

        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, kMaxPossibleSpeed);

        // passing back the math from kinematics to the swerves themselves.
        m_frontLeft.setDesiredState(swerveModuleStates[0]);
        m_frontRight.setDesiredState(swerveModuleStates[1]);
        m_backLeft.setDesiredState(swerveModuleStates[2]);
        m_backRight.setDesiredState(swerveModuleStates[3]);

        SmartDashboard.putNumber("desired X Speed", xSpeed);
        SmartDashboard.putNumber("desired Y Speed", ySpeed);
        SmartDashboard.putNumber("desired Rot Speed", Math.toDegrees(rotSpeed));
        updateOdometry();
    }

    public void driveWithChasisSpeeds(ChassisSpeeds chassisSpeeds) {
        SwerveModuleState[] swerveModuleStates = m_kinematics.toSwerveModuleStates(chassisSpeeds);

        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, kMaxPossibleSpeed);
        ;
        SmartDashboard.putNumber("desired X Speed", chassisSpeeds.vxMetersPerSecond);
        SmartDashboard.putNumber("desired Y Speed", chassisSpeeds.vyMetersPerSecond);
        SmartDashboard.putNumber("desired Rot Speed", Math.toDegrees(chassisSpeeds.omegaRadiansPerSecond));

        // passing back the math from kinematics to the swerves themselves.
        m_frontLeft.setDesiredState(swerveModuleStates[0]);
        m_frontRight.setDesiredState(swerveModuleStates[1]);
        m_backLeft.setDesiredState(swerveModuleStates[2]);
        m_backRight.setDesiredState(swerveModuleStates[3]);
        updateOdometry();
    }

    PIDController pieceLockPID = new PIDController(0.05, 0, 0); //todo tune perfectly.
    public void lockPiece(double xSpeed, double ySpeed, double rotSpeed, boolean fieldRelative, boolean hardLocked) {
        SwerveModuleState[] swerveModuleStates; //MAKE SURE swervestates can be init like this with this kinda array
        if(hasTarget()) {
                rotSpeed = -pieceLockPID.calculate(getGyroYawRotation2d().getDegrees(), getRobotRotToTarg());
                if(hardLocked) {
                        swerveModuleStates = m_kinematics.toSwerveModuleStates(
                                new ChassisSpeeds(xSpeed, 0, rotSpeed)
                        );
                }
                else {
                        swerveModuleStates = m_kinematics.toSwerveModuleStates(
                                ChassisSpeeds.fromFieldRelativeSpeeds(
                                        xSpeed, ySpeed, rotSpeed, getGyroYawRotation2d()
                                )
                        ); 
                }
        }
        else {
                swerveModuleStates = m_kinematics.toSwerveModuleStates(
                fieldRelative
                        ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rotSpeed,
                                getGyroYawRotation2d())
                        : new ChassisSpeeds(xSpeed, ySpeed, rotSpeed));
        }

        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, kMaxPossibleSpeed);

        // passing back the math from kinematics to the swerves themselves.
        m_frontLeft.setDesiredState(swerveModuleStates[0]);
        m_frontRight.setDesiredState(swerveModuleStates[1]);
        m_backLeft.setDesiredState(swerveModuleStates[2]);
        m_backRight.setDesiredState(swerveModuleStates[3]);

        SmartDashboard.putNumber("desired X Speed", xSpeed);
        SmartDashboard.putNumber("desired Y Speed", ySpeed);
        SmartDashboard.putNumber("desired Rot Speed", Math.toDegrees(rotSpeed));
        updateOdometry();
    }


    public Pose2d getRoboPose2d() {
        return m_odometry.getPoseMeters();
    }

    /**
     * THIS IS IN DEGREES
     * Triangulates position of robot knowing the distance between two april tags seen by the camera.
     * @param length
     * @param angle1
     * @param angle2
     * @return
     */
    public Pose2d calcRoboPose2dWithVision(double length, double angle1, double angle2) {
        double L = length; //dist between the two april tags
        double a1 = angle1; //angle (from the camera) of the close april tag (a1) and the far april tag (a2)
        double a2 = angle2;
        double gyroOffset = 0;
        double roboAngle = -m_gyro.getYaw() + gyroOffset; //angle of the robot (0 degrees = facing the drivers)

        double X = (L * Math.sin(Math.toRadians(90 + roboAngle + a2)) * Math.sin(Math.toRadians(90 - roboAngle - a1)))
                        /Math.sin(Math.toRadians(Math.abs(a2 - a1)));

        double Y = (L * Math.sin(Math.toRadians(90 + roboAngle + a2)) * Math.cos(Math.toRadians(90 - roboAngle - a1)))
                        /Math.sin(Math.toRadians(Math.abs(a2 - a1)));

        return new Pose2d(X, Y, getGyroYawRotation2d());
    }

    public void stopDriving() {
        System.out.println("I AINT DRIVING");
        SwerveModuleState[] swerveModuleStates = m_kinematics.toSwerveModuleStates(
        ChassisSpeeds.fromFieldRelativeSpeeds(
                0,
                0,
                0,
                getGyroYawRotation2d()));

        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, kMaxPossibleSpeed);

        // passing back the math from kinematics to the swerves themselves.
        m_frontLeft.setDesiredState(swerveModuleStates[0]);
        m_frontRight.setDesiredState(swerveModuleStates[1]);
        m_backLeft.setDesiredState(swerveModuleStates[2]);
        m_backRight.setDesiredState(swerveModuleStates[3]);
    }

    /** Updates the field relative position of the robot. */
    public void updateOdometry() {
        SmartDashboard.putNumber("Desired Angle", getRobotRotToTarg());
        SmartDashboard.putNumber("ID", LimelightHelpers.getFiducialID(""));
        SmartDashboard.putNumber("WTF the gyro is", getGyroYawRotation2d().getDegrees());
        m_odometry.update(
                getGyroYawRotation2d(),
                getModulePositions());
        
        // getting velocity vectors from each module
        SwerveModuleState frontLeftState = m_frontLeft.getState();
        SwerveModuleState frontRightState = m_frontRight.getState();
        SwerveModuleState backLeftState = m_backLeft.getState();
        SwerveModuleState backRightState = m_backRight.getState();

        // Converting module speeds to chassis speeds
        m_chassisSpeeds = m_kinematics.toChassisSpeeds(
                frontLeftState, frontRightState, backLeftState, backRightState);
        robotFieldPosition = getRoboPose2d();

        // Getting X Y R vector speeds
        double forward = m_chassisSpeeds.vxMetersPerSecond;
        double sideways = m_chassisSpeeds.vyMetersPerSecond;
        double angular = m_chassisSpeeds.omegaRadiansPerSecond;

        SmartDashboard.putNumber("real X speed", forward);
        SmartDashboard.putNumber("real Y speed", sideways);
        SmartDashboard.putNumber("real Rot speed", Math.toDegrees(angular));
        SmartDashboard.putNumber("real X Pos", robotFieldPosition.getX());
        SmartDashboard.putNumber("real Y Pos", robotFieldPosition.getY());
        SmartDashboard.putNumber("real Rot", robotFieldPosition.getRotation().getDegrees());
    }
}
