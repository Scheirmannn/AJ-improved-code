package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class GoBackTime {

    public static Command get(DriveSubsystem drive, ShooterSubsystem shooter) {
        Command sequence = Commands.sequence(
            new RunCommand(() -> drive.drive(-0.3, 0, 0, false), drive)
                .withTimeout(1.0),

            new InstantCommand(() -> drive.drive(0, 0, 0, false), drive),

            shooter.fullShootCommand(3800, 4600)
                .withTimeout(8.0),

            shooter.fullStopCommand()
        );

        return sequence.finallyDo(() -> {
            drive.drive(0, 0, 0, false);
            shooter.fullStopCommand().schedule();
        });
    }
}
