// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.Controller;
import frc.robot.Constants.Deadbands;
import frc.robot.Subsystems.CameraSubsystem.CameraSourceOption;
import frc.robot.Subsystems.CameraSubsystem;
import frc.robot.Subsystems.Drivetrain;
import frc.robot.Commands.Drive;
import frc.robot.Commands.StopDrive;
import frc.robot.Commands.lockTarget;
import frc.robot.Commands.resetOdo;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  private final SendableChooser<Command> autoChooser;
  private final XboxController m_badcontroller = new XboxController(Controller.kDriveController);
  private final CommandXboxController m_controller = new CommandXboxController(Controller.kDriveController);
    public final Drivetrain m_swerve = new Drivetrain();
  private final CameraSubsystem m_cameraSubsystem = new CameraSubsystem();
    Command driveCommand;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Xbox controllers return negative values when we push forward.   
    driveCommand = new Drive(m_swerve);

    m_swerve.setDefaultCommand(driveCommand);
    
    autoChooser = AutoBuilder.buildAutoChooser(); // Default auto will be `Commands.none()'
    // Create choices for autonomous functions in the Smart Dashboard
    SmartDashboard.putData("Auto Mode", autoChooser);
    // Register named commands
    NamedCommands.registerCommand("StopDrive", new StopDrive(m_swerve));

    // Configure the trigger bindings
    pathPlannerStuff();
    configureBindings();
    autoChooser.setDefaultOption("Autonomous Command", path1);
    // Configure the button bindings
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    Command lock = new lockTarget(m_swerve);
    m_controller.leftBumper().onTrue(lock);
   m_controller.rightBumper().onTrue((new resetOdo(m_swerve)));
    m_controller.x().whileTrue(pathfindAmp);
    /*
         * Y = forward camera
         * A = back camera
         */
        m_controller.y().onTrue(this.m_cameraSubsystem.setSourceCommand(CameraSourceOption.USB_CAMERA)
                .alongWith(this.m_swerve.setDirectionOptionCommand(Drivetrain.DirectionOption.FORWARD)));
        m_controller.b().onTrue(this.m_cameraSubsystem.setSourceCommand(CameraSourceOption.FISHEYE));
        m_controller.a().onTrue(this.m_cameraSubsystem.setSourceCommand(CameraSourceOption.LIMELIGHT)
                .alongWith(this.m_swerve.setDirectionOptionCommand(Drivetrain.DirectionOption.BACKWARD)));
        m_controller.povUp().toggleOnTrue(this.m_swerve.toggleFieldRelativeCommand());

  }

  Command path1;
  Command pathfindAmp;
  private void pathPlannerStuff() {
    // Add a button to run the example auto to SmartDashboard, this will also be in the auto chooser built above
    //SmartDashboard.putData("Example Auto", AutoBuilder.buildAuto("Example Auto"));
    // Add a button to run a simple example path
    path1 = AutoBuilder.buildAuto("Vision Test");
    autoChooser.addOption("path", path1);
    // Load the path we want to pathfind to and follow
    PathPlannerPath path = PathPlannerPath.fromPathFile("Score Amp");
    // Create the constraints to use while pathfinding. The constraints defined in the path will only be used for the path.
    PathConstraints constraints = new PathConstraints(
      1, 3.0,
      Units.degreesToRadians(540), Units.degreesToRadians(720));
    // Since AutoBuilder is configured, we can use it to build pathfinding commands
    pathfindAmp = AutoBuilder.pathfindThenFollowPath(
      path,
      constraints,
      0.0 // Rotation delay distance in meters. This is how far the robot should travel before attempting to rotate.
    );
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return path1;
  }

  public Command getTeleOpCommand() {
    return driveCommand;
  }

  /**
   * Pathfind to the Amp
   */
  public Command pathfindAmp() {
    return pathfindAmp;
  }
}
