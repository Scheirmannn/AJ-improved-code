package frc.robot;

import choreo.Choreo;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

import java.util.List;
import java.util.Optional;

public final class Autos {

    private Autos() {
    }

    private static Command buildChoreoAuto(DriveSubsystem drive, String trajectoryName) {
        Optional<Trajectory<SwerveSample>> trajectory = Choreo.loadTrajectory(trajectoryName);

        if (trajectory.isEmpty()) {
            return Commands.print("[Autos] WARNING: Trajectory '" + trajectoryName + "' not found!");
        }

        List<SwerveSample> samples = trajectory.get().samples();

        return Commands.sequence(
                Commands.runOnce(() -> drive.resetOdometry(samples.get(0).getPose())),
                Commands.sequence(samples.stream()
                        .map(sample -> Commands.runOnce(() -> drive.followTrajectory(sample), drive)
                                .andThen(Commands.waitSeconds(0.02)))
                        .toArray(Command[]::new)),
                Commands.runOnce(() -> drive.stopModules(), drive));
    }

    public static Command followChoreoTrajectory(DriveSubsystem drive, String trajectoryName) {
        return buildChoreoAuto(drive, trajectoryName);
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