package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class GoBackTime extends Command {

    private final DriveSubsystem m_drive;
    private final ShooterSubsystem m_shooter;

    private Command m_autoSequence;

    public GoBackTime(DriveSubsystem drive, ShooterSubsystem shooter) {
        m_drive = drive;
        m_shooter = shooter;
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        m_autoSequence = Commands.sequence(    
            new RunCommand(() -> m_drive.drive(-0.3, 0, 0, false), m_drive).withTimeout(1.0),

            new InstantCommand(() -> m_drive.drive(0, 0, 0, false), m_drive),

            m_shooter.fullShootCommand(3800, 4600).withTimeout(8.0),

            m_shooter.fullStopCommand()
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