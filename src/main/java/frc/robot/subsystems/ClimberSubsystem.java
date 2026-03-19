package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubsystem extends SubsystemBase {
/* 
    private final DoubleSolenoid climberSolenoid;
    private final Compressor compressor;

    public boolean m_climbed = false;


    public ClimberSubsystem() {
        compressor = new Compressor(PneumaticsModuleType.REVPH);
        compressor.enableDigital();
        climberSolenoid = new DoubleSolenoid(2, PneumaticsModuleType.REVPH, 7, 6);

        SmartDashboard.putBoolean("Is at Pressure", false);

    }
    
    public void setClimbUp() {
        climberSolenoid.set(DoubleSolenoid.Value.kForward);
        m_climbed = true;
    }

    public void setClimbDown() {
        climberSolenoid.set(DoubleSolenoid.Value.kReverse);
        m_climbed = false;
    }

     public void setClimbStop() {
        climberSolenoid.set(DoubleSolenoid.Value.kOff);
    }

    public Command climbStopCommand () {
        return new InstantCommand(() -> setClimbStop(), this);
    }

    public Command climbDownCommand(double time) {
        return Commands.sequence(
            new InstantCommand(() -> {
                if (m_climbed) {
                    setClimbDown();
                }
            }, this),
            Commands.waitSeconds(time),
            climbStopCommand()
        );
    }

    public Command climbUpCommand(double time) {
      return Commands.sequence(
            new InstantCommand(() -> {
                if (!m_climbed) {
                    setClimbUp();
                }
            }, this),
            Commands.waitSeconds(time),
            climbStopCommand()
        ); 
    }

    public boolean isAtPressure() {
        return compressor.getPressureSwitchValue();
    }

    public void enableCompressor() {
        compressor.enableDigital();
    }

    public void disableCompressor() {
        compressor.disable();
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Is at Pressure", isAtPressure());
    }
*/
}
