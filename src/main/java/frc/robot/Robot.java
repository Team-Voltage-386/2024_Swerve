// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.Deadbands;
import frc.robot.Commands.Drive;
import frc.robot.Constants.Controller;

public class Robot extends TimedRobot {
    
    private final XboxController m_controller = new XboxController(Controller.kDriveController);
    private final Drivetrain m_swerve = new Drivetrain();
    private final RobotContainer m_containter = new RobotContainer(m_swerve);

    // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
    private final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(Controller.kRateLimitXSpeed);
    private final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(Controller.kRateLimitYSpeed);
    private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(Controller.kRateLimitRot);

    private Command autonomousCommand;
    private Command driveCommand; //todo

    @Override
    public void robotInit() {
        autonomousCommand = m_containter.getAutonomousCommand();
        driveCommand = new Drive(m_swerve, xSpeed, ySpeed, rot, true);
    }

    @Override
    public void autonomousInit() {
        autonomousCommand.schedule();
    }

    @Override
    public void teleopInit() {
        autonomousCommand.cancel();
        m_swerve.setDefaultCommand(driveCommand);
    }

    @Override
    public void autonomousPeriodic() {
        if(autonomousCommand.isFinished()) {
            System.out.println("FINISHED AUTO!");
        }
    } 

    @Override
    public void teleopPeriodic() {
        if(m_controller.getAButtonPressed())
                m_containter.pathfindAmp().schedule();
            else
                readControllers(true);
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }

    @Override
    public void disabledPeriodic() {
        // Only needed when measuring and configuring the encoder offsets. Can comment
        // out when not used
        //m_swerve.print();
    }

    double xSpeed;
    double ySpeed;
    double rot;

    private void readControllers(boolean fieldRelative) {
        // Get the x speed. We are inverting this because Xbox controllers return
        // negative values when we push forward.
        xSpeed = -m_xspeedLimiter
                .calculate(MathUtil.applyDeadband(m_controller.getLeftY(), Deadbands.kLeftJoystickDeadband))
                * Constants.Controller.kMaxNecessarySpeed;

        // Get the y speed or sideways/strafe speed. We are inverting this because
        // we want a positive value when we pull to the left. Xbox controllers
        // return positive values when you pull to the right by default.
        ySpeed = -m_yspeedLimiter
                .calculate(MathUtil.applyDeadband(m_controller.getLeftX(), Deadbands.kLeftJoystickDeadband))
                * Constants.Controller.kMaxNecessarySpeed;

        // Get the rate of angular rotation. We are inverting this because we want a
        // positive value when we pull to the left (remember, CCW is positive in
        // mathematics). Xbox controllers return positive values when you pull to
        // the right by default.
        rot = -m_rotLimiter
                .calculate(MathUtil.applyDeadband(m_controller.getRightX(), Deadbands.kRightJoyStickDeadband))
                * Drivetrain.kMaxAngularSpeed;

        if(m_controller.getLeftBumper()){
            if(m_controller.getLeftTriggerAxis() > 0.25)
                m_swerve.lockPiece(xSpeed, ySpeed, rot, fieldRelative, true);
            else
                m_swerve.lockPiece(xSpeed, ySpeed, rot, fieldRelative, false);
        }
        else{
            m_swerve.drive(xSpeed, ySpeed, rot, fieldRelative);
        }

        if (m_controller.getRightBumperPressed()) {
            m_swerve.resetOdo();
            m_swerve.resetGyro();
        }
    }
}
