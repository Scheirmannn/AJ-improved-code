package frc.robot.subsystems;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkBase.ControlType;

import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Command;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;

import java.util.function.DoubleSupplier;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;

import frc.robot.Configs;

public class ShooterSubsystem extends SubsystemBase {

    private final SparkMax shooterMotor;
    private final SparkMax backMotor;
    private final SparkMax rollerMotor;

    private final RelativeEncoder shooterEncoder;

    private Vision m_vision = null;

    private static final InterpolatingDoubleTreeMap shooterRPMMap = new InterpolatingDoubleTreeMap();
    private static final InterpolatingDoubleTreeMap backrollerRPMMap = new InterpolatingDoubleTreeMap();


    static {
        //Distance(meters) -> shooter power
        //Vortex Free Speed is ~6784 RPM, so dont exceed this
        //Both wheels equal at close range, diff increases at distance
        //to add more arc
        //Change later
        shooterRPMMap.put(1.04, 3000.0);
        shooterRPMMap.put(2.4, 4000.0);
        shooterRPMMap.put(3.0, 4250.0);
        shooterRPMMap.put(4.0, 4600.0);
        shooterRPMMap.put(5.0, 5500.0);

        //Distance(meters) -> backroller power
        //Tune the ratio between shooter and backroller to shape the shot arc
        //Change later
        backrollerRPMMap.put(1.04, 5500.0); //eqaul at close range = flatter arc
        backrollerRPMMap.put(2.4, 5000.0); 
        backrollerRPMMap.put(3.0, 5000.0);
        backrollerRPMMap.put(4.0, 4500.0);
        backrollerRPMMap.put(5.0, 4500.0);   
    }
    
    public ShooterSubsystem(int shooterMotorId, int backMotorId, int rollerMotorId) {
        shooterMotor = new SparkMax(shooterMotorId, SparkMax.MotorType.kBrushless);
        backMotor = new SparkMax(backMotorId, SparkMax.MotorType.kBrushless);
        rollerMotor = new SparkMax(rollerMotorId, SparkMax.MotorType.kBrushless);

        shooterMotor.configure(Configs.ShooterSubsystem.SHOOTER_CONFIG, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        backMotor.configure(Configs.ShooterSubsystem.BACKROLLER_CONFIG, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        rollerMotor.configure(Configs.HopperSubsystem.INDEXER_CONFIG, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        shooterEncoder = shooterMotor.getEncoder();

        SmartDashboard.putBoolean("Shooter at Speed", false);
    }
    //Moved distance calcs to outside the shootCommand
    public double desiredShooterRPM(DoubleSupplier distanceMeters) {
        double dist = distanceMeters.getAsDouble();

        // Clamp distance to table bouds so it doesnt extrapolate wildly
        dist = Math.max(1.0, Math.min(5.0, dist));

        // Shot selection label for dashboard
        String shotLabel;
        if (dist < 1.7) {
            shotLabel = "Close";
        } else if (dist < 2.7) {
            shotLabel = "Mid-Close";
        } else if (dist < 3.5) {
            shotLabel = "Mid";
        } else if (dist < 4.5) {
            shotLabel = "Mid-Far";
        } else {
            shotLabel = "Far";
        }
        SmartDashboard.putString("Shot Selection", shotLabel);

        return shooterRPMMap.get(dist);
    }

    public double desiredBackRPM(DoubleSupplier distanceMeters) {
        double dist = distanceMeters.getAsDouble();

        // Clamp distance to table bouds so it doesnt extrapolate wildly
        dist = Math.max(1.0, Math.min(5.0, dist));

        // Shot selection label for dashboard
        String shotLabel;
        if (dist < 1.7) {
            shotLabel = "Close";
        } else if (dist < 2.7) {
            shotLabel = "Mid-Close";
        } else if (dist < 3.5) {
            shotLabel = "Mid";
        } else if (dist < 4.5) {
            shotLabel = "Mid-Far";
        } else {
            shotLabel = "Far";
        }
        SmartDashboard.putString("Shot Selection", shotLabel);

        return backrollerRPMMap.get(dist);
    }
    //added vision setter for shooter
    public void useVision(Vision vision) {
        m_vision = vision;
    }
    //shooter velo methods to work with roller wait
    public double getShooterVelocity() {
        return shooterEncoder.getVelocity();
    }

    public boolean isAtSpeed(double shooterRPM) {
        return getShooterVelocity() >= (shooterRPM * 0.95);
    }

    public boolean isAtVisionSpeed() {
        return getShooterVelocity() >= (desiredShooterRPM(() -> m_vision.getHubDistance()) * 0.95);
    }

    public boolean isShooterRunning() {
        return getShooterVelocity() > 1;
    }
    //rpm controls
    public void setShooterVelo(double rpm) {
        shooterMotor.getClosedLoopController().setSetpoint(rpm, ControlType.kVelocity);
    }

    public void setBackVelo(double rpm) {
        backMotor.getClosedLoopController().setSetpoint(rpm, ControlType.kVelocity);
    }

    public void setRollerVelo(double rpm) {
        rollerMotor.getClosedLoopController().setSetpoint(rpm, ControlType.kVelocity);
    }

    public void gateStop() {
        rollerMotor.stopMotor();
    }

    public Command shootCommand(double shooterRPM, double backRPM) {
        return new RunCommand(() -> {
            setShooterVelo(shooterRPM);
            setBackVelo(backRPM);
        });
    }

    public Command shootVisionCommand(DoubleSupplier distanceMeters) {
        return new RunCommand(() -> {
            double shooterRPM = desiredShooterRPM(distanceMeters);
            double backRPM = desiredBackRPM(distanceMeters);

            setShooterVelo(shooterRPM);
            setBackVelo(backRPM);

            SmartDashboard.putNumber("Shooter/Distance (m)", distanceMeters.getAsDouble());
            SmartDashboard.putNumber("Shooter/Flywheel RPM", shooterRPM);
            SmartDashboard.putNumber("Shooter/Backroller RPM", backRPM);
        });
    }

    public Command rollerStartCommand() {
        return new InstantCommand(() -> setRollerVelo(3000), this);
    }

    public Command rollerReverseCommand() {
        return new InstantCommand(() -> setRollerVelo(-3000), this);
    }

    public Command rollerWaitommand(double shooterRPM) {
        return Commands.sequence(
                Commands.waitUntil(() -> isAtSpeed(shooterRPM)),
                Commands.waitSeconds(0.5),
                rollerStartCommand());
    }

    public Command rollerWaitVisionCommand() {
        return Commands.sequence(
            Commands.waitUntil(this::isAtVisionSpeed),
            Commands.waitSeconds(0.25),
            rollerStartCommand()
        );
    }

    public Command rollerStopCommand() {
        return new InstantCommand(() -> setRollerVelo(0) , this);
    }

    public Command fullStopCommand() {
        return new InstantCommand(() -> {
            gateStop();
            setBackVelo(0);
            setShooterVelo(0);
        }, this);
    }

    public Command fullShootCommand(double shooterRPM, double backRPM) {
        return Commands.parallel(
            shootCommand(shooterRPM, backRPM),
            rollerWaitommand(shooterRPM)
        );
    }

    public Command fullShootVisionCommand() {
        return Commands.parallel(
            shootVisionCommand(() -> m_vision.getHubDistance()),
            rollerWaitVisionCommand()
        );
    }

    @Override
    public void periodic() {
       SmartDashboard.putNumber("shooter RPM", getShooterVelocity());
    }
}

