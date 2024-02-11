// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Commands.RumblePulseCommand;

public class RumbleSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  private CommandXboxController m_controller;
  private double m_rumbleRightLastSetValue;
  private double m_rumbleLeftLastSetValue;

  public RumbleSubsystem(CommandXboxController controller) {
    this.m_controller = controller;
    m_rumbleRightLastSetValue = 0.0;
    m_rumbleLeftLastSetValue = 0.0;

  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  public void setRumble(RumbleType type, double value) {
    m_controller.getHID().setRumble(type, value);
    updateRumbleValues(type, value);
  }

  public void updateRumbleValues(RumbleType type, double value) {
    switch (type) {
      case kBothRumble:
        m_rumbleRightLastSetValue = value;
        m_rumbleLeftLastSetValue = value;
        break;
      case kLeftRumble:
        m_rumbleLeftLastSetValue = value;
        break;
      case kRightRumble:
        m_rumbleRightLastSetValue = value;
        break;
      default:
        assert(false);
        break;
    }
  }

   public Command setRumbleCommand(RumbleType type, double value, double timeBetweenPulses) {
      return new RumblePulseCommand(type, value, timeBetweenPulses, this);
   }

   public boolean isRumbling(RumbleType type) {
    switch (type) {
      case kBothRumble:
        return this.isRumbling(RumbleType.kLeftRumble) && this.isRumbling(RumbleType.kRightRumble);
      case kLeftRumble:
        return m_rumbleLeftLastSetValue > 0.0;
      case kRightRumble:
        return m_rumbleRightLastSetValue > 0.0;
      default:
        assert(false);
        return false;
    }
   }
 

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
