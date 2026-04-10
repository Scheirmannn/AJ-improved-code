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

	private final SparkMax shooterMotor1;
	private final SparkMax shooterMotor2;
	private final SparkMax backMotor;
	private final SparkMax rollerMotor1;
	private final SparkMax rollerMotor2;

	private final RelativeEncoder shooterEncoder1;
	private final RelativeEncoder shooterEncoder2;

	private Vision m_vision = null;

	private static final InterpolatingDoubleTreeMap shooterRPMMap = new InterpolatingDoubleTreeMap();
	private static final InterpolatingDoubleTreeMap backrollerRPMMap = new InterpolatingDoubleTreeMap();

	static {
		// Distance(meters) -> shooter power
		// Vortex Free Speed is ~6784 RPM, so dont exceed this
		// Both wheels equal at close range, diff increases at distance
		// to add more arc
		// Change later
		shooterRPMMap.put(1.0, 2750.0);
		shooterRPMMap.put(1.5, 2875.0);
		shooterRPMMap.put(2.0, 3200.0);
		shooterRPMMap.put(2.5, 3400.0);
		shooterRPMMap.put(3.0, 3800.0);

		// Distance(meters) -> backroller power
		// Tune the ratio between shooter and backroller to shape the shot arc
		// Change later
		backrollerRPMMap.put(1.0, 4500.0); // eqaul at close range = flatter arc
		backrollerRPMMap.put(1.5, 5000.0);
		backrollerRPMMap.put(2.0, 4200.0);
		backrollerRPMMap.put(2.5, 4600.0);
		backrollerRPMMap.put(3.0, 4300.0);

	}

	public ShooterSubsystem() {
		shooterMotor1 = new SparkMax(6, SparkMax.MotorType.kBrushless);
		shooterMotor2 = new SparkMax(7, SparkMax.MotorType.kBrushless);
		backMotor = new SparkMax(19, SparkMax.MotorType.kBrushless);
		rollerMotor1 = new SparkMax(20, SparkMax.MotorType.kBrushless);
		rollerMotor2 = new SparkMax(21, SparkMax.MotorType.kBrushless);

		shooterMotor1.configure(Configs.Shooter.SHOOTER_CONFIG, ResetMode.kResetSafeParameters,
				PersistMode.kPersistParameters);
		shooterMotor2.configure(Configs.Shooter.SHOOTER_CONFIG, ResetMode.kResetSafeParameters,
				PersistMode.kPersistParameters);
		backMotor.configure(Configs.Shooter.BACKROLLER_CONFIG, ResetMode.kResetSafeParameters,
				PersistMode.kPersistParameters);
		rollerMotor1.configure(Configs.HopperSubsystem.INDEXER_CONFIG, ResetMode.kResetSafeParameters,
				PersistMode.kPersistParameters);
		rollerMotor2.configure(Configs.HopperSubsystem.INDEXER_CONFIG, ResetMode.kResetSafeParameters,
				PersistMode.kPersistParameters);

		shooterEncoder1 = shooterMotor1.getEncoder();
		shooterEncoder2 = shooterMotor2.getEncoder();

		SmartDashboard.putBoolean("Shooter at Speed", false);
	}

	// Moved distance calcs to outside the shootCommand
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

	// added vision setter for shooter
	public void useVision(Vision vision) {
		m_vision = vision;
	}

	// shooter velo methods to work with roller wait
	public double getShooterVelocity() {
		double RPM = (Math.abs(shooterEncoder1.getVelocity()) + Math.abs(shooterEncoder2.getVelocity())) / (2.0);
		return RPM;
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

	// rpm controls
	public void setShooterVelo(double rpm) {
		shooterMotor1.getClosedLoopController().setSetpoint(rpm, ControlType.kVelocity);
		shooterMotor2.getClosedLoopController().setSetpoint(-rpm, ControlType.kVelocity);
	}

	public void setBackVelo(double rpm) {
		backMotor.getClosedLoopController().setSetpoint(rpm, ControlType.kVelocity);
	}

	public void setRollerVelo(double rpm) {
		rollerMotor1.getClosedLoopController().setSetpoint(rpm, ControlType.kVelocity);
		rollerMotor2.getClosedLoopController().setSetpoint(rpm, ControlType.kVelocity);
	}

	public void rollerStop() {
		rollerMotor1.stopMotor();
		rollerMotor2.stopMotor();
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
				rollerStartCommand());
	}

	public Command fullStopCommand() {
		return new InstantCommand(() -> {
			rollerStop();
			setBackVelo(0);
			setShooterVelo(0);
		}, this);
	}

	public Command fullShootCommand(double shooterRPM, double backRPM) {
		return Commands.parallel(
				shootCommand(shooterRPM, backRPM),
				rollerWaitommand(shooterRPM));
	}

	public Command fullShootVisionCommand() {
		return Commands.parallel(
				shootVisionCommand(() -> m_vision.getHubDistance()),
				rollerWaitVisionCommand());
	}

	@Override
	public void periodic() {
		SmartDashboard.putNumber("shooter RPM", getShooterVelocity());
	}
}
