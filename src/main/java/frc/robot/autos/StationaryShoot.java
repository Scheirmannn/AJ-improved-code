package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.DriveSubsystem;

public class StationaryShoot extends Command {

    private final DriveSubsystem m_drive;
    private final ShooterSubsystem m_shooter;

    private Command m_autoSequence;

    public StationaryShoot(DriveSubsystem drive, ShooterSubsystem shooter) {
        m_drive = drive;
        m_shooter = shooter;
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        m_autoSequence = Commands.sequence(
            m_shooter.fullShootCommand(3800.0, 4600.0).withTimeout(15)
            
            );
        
            CommandScheduler.getInstance().schedule(m_autoSequence);
    }

    @Override
    public void end(boolean interrupted) {
        if (m_autoSequence != null)
            m_autoSequence.cancel();
        m_drive.drive(0, 0, 0, false);
        CommandScheduler.getInstance().schedule(m_shooter.fullStopCommand());
    }

    @Override
    public boolean isFinished() {
        return m_autoSequence != null && !m_autoSequence.isScheduled();
    }
}