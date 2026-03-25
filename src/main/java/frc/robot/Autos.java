package frc.robot;

import choreo.auto.AutoFactory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.Vision;

public class Autos {
    private final AutoFactory factory;
    private final DriveSubsystem drivetrain;
    private final ShooterSubsystem shooter;
    private final HopperSubsystem hopper;
    private final IntakeSubsystem intake;
    private final Vision vision;

    public Autos (DriveSubsystem drivetrain, ShooterSubsystem shooter, HopperSubsystem hopper, IntakeSubsystem intake, Vision vision) {
        this.drivetrain = drivetrain;
        this.shooter = shooter;
        this.hopper = hopper;
        this.intake = intake;
        this.vision = vision;

        factory = new AutoFactory(
            drivetrain::getPose,
            drivetrain:: resetOdometry,
            drivetrain::followTrajectory,
            true,
            drivetrain);
    }

   
    // ----------------------------------------------------------------------------
    public Command newPath() {
        var routine = factory.newRoutine("NewAuto");
        var traj1 = routine.trajectory("Driveback_1m"); // Default auto

        routine.active().onTrue(
            Commands.sequence(
                // Correct robot gyro
                Commands.runOnce(()-> {
                    drivetrain.zeroHeading(); // Added zeroheading
                    if (traj1.getInitialPose().get() != null) {
                        drivetrain.resetOdometry(traj1.getInitialPose().get()); // Added reset odometry
                    } else {
                        drivetrain.resetOdometry(new Pose2d()); // If there is no initial pose, create new pose
                    }
                }, drivetrain),

                // Call trajectory
                traj1.cmd(),
                Commands.runOnce(()-> drivetrain.drive(0, 0, 0, true), drivetrain),

                shooter.reverseShot(), Commands.waitSeconds(.5),

                shooter.stopShotCommand(), Commands.waitSeconds(.3),

                shooter.shootFixedAutoCommand(), Commands.waitSeconds(1),

                hopper.rollCommand(), Commands.waitSeconds(8),

                shooter.stopShotCommand(),
                hopper.stopRollerCommand()

                // After every command, log finished in console
            )
        );    
        return routine.cmd();
    } 
}
