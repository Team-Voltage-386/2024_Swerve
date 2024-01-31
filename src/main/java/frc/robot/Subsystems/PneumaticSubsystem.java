package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PneumaticSubsystem extends SubsystemBase {
    private DoubleSolenoid m_handOff;
    private boolean m_handOffDeployed = false;

    public PneumaticSubsystem() {
        this.m_handOff = new DoubleSolenoid(0, PneumaticsModuleType.CTREPCM, 2, 3);
    }

    public void enableLift() {
        this.m_handOff.set(Value.kForward);
        this.liftOut(false);
    }

    public void disableLift() {
        this.m_handOff.set(Value.kReverse);
        this.liftOut(true);
    }

    public boolean getHandOffDeployed() {
        return m_handOffDeployed;
    }

    public void liftOut(boolean isLiftOut) {
        this.m_handOffDeployed = isLiftOut;
    }
}
