package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.ShooterSubsystem;

public class StationaryShoot {

    public static Command get(ShooterSubsystem shooter) {
        Command sequence = Commands.sequence(
                shooter.fullShootCommand(3800, 4600));

        return sequence.finallyDo(() -> {
            shooter.fullStopCommand().schedule();
        });
    }
}
