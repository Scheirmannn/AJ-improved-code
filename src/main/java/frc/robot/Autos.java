package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public final class Autos {

    private Autos() {
    }

    public static Command goBackTime(DriveSubsystem drive, ShooterSubsystem shooter) {
        return Commands.sequence(
                Commands.run(() -> drive.drive(-0.3, 0, 0, false), drive)
                        .withTimeout(1.0),
                Commands.runOnce(() -> drive.stopModules(), drive),
                shooter.fullShootCommand(3800, 4600)
                        .withTimeout(8.0),
                shooter.fullStopCommand())

                .finallyDo(() -> {
                    drive.stopModules();
                    shooter.setShooterVelo(0);
                    shooter.setBackVelo(0);
                    shooter.rollerStop();
                });
    }

    public static Command stationaryShoot(ShooterSubsystem shooter) {
        return Commands.sequence(
                shooter.fullShootCommand(3800, 4600)
                        .withTimeout(15),
                shooter.fullStopCommand())

                .finallyDo(() -> {
                    shooter.setShooterVelo(0);
                    shooter.setBackVelo(0);
                    shooter.rollerStop();
                });
    }

    public static Command doNothing() {
        return Commands.none();
    }
}