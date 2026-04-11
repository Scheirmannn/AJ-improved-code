package frc.robot.subsystems;

import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;

import com.studica.frc.AHRS;

import choreo.trajectory.SwerveSample;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.RobotBase;

public class DriveSubsystem extends SubsystemBase {
	private final MAXSwerveModule m_frontLeft = new MAXSwerveModule(
			DriveConstants.kFrontLeftDrivingCanId,
			DriveConstants.kFrontLeftTurningCanId,
			DriveConstants.kFrontLeftChassisAngularOffset);

	private final MAXSwerveModule m_frontRight = new MAXSwerveModule(
			DriveConstants.kFrontRightDrivingCanId,
			DriveConstants.kFrontRightTurningCanId,
			DriveConstants.kFrontRightChassisAngularOffset);

	private final MAXSwerveModule m_rearLeft = new MAXSwerveModule(
			DriveConstants.kRearLeftDrivingCanId,
			DriveConstants.kRearLeftTurningCanId,
			DriveConstants.kBackLeftChassisAngularOffset);

	private final MAXSwerveModule m_rearRight = new MAXSwerveModule(
			DriveConstants.kRearRightDrivingCanId,
			DriveConstants.kRearRightTurningCanId,
			DriveConstants.kBackRightChassisAngularOffset);

	public final AHRS m_gyro = new AHRS(AHRS.NavXComType.kUSB1);
	public SwerveDrivePoseEstimator poseEstimator;
	private final SwerveDriveOdometry m_odometry;

	private double m_simGyroAngle = 0.0;
	private ChassisSpeeds m_lastChassisSpeeds = new ChassisSpeeds();

	private final PIDController xController = new PIDController(8.0, 0.0, 0.0);
	private final PIDController yController = new PIDController(8.0, 0, 0);
	private final PIDController headingController = new PIDController(7.5, 0, 0);

	public DriveSubsystem() {
		headingController.enableContinuousInput(-Math.PI, Math.PI);
		HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_MaxSwerve);

		var stateStdDevs = VecBuilder.fill(0.1, 0.1, 0.1);
		var visionStdDevs = VecBuilder.fill(1, 1, 1);
		poseEstimator = new SwerveDrivePoseEstimator(
				DriveConstants.kDriveKinematics,
				Rotation2d.fromDegrees(m_gyro.getYaw()),
				new SwerveModulePosition[] {
						m_frontLeft.getPosition(),
						m_frontRight.getPosition(),
						m_rearLeft.getPosition(),
						m_rearRight.getPosition()
				},
				new Pose2d(),
				stateStdDevs,
				visionStdDevs);

		m_odometry = new SwerveDriveOdometry(
				DriveConstants.kDriveKinematics,
				Rotation2d.fromDegrees(m_gyro.getYaw()),
				new SwerveModulePosition[] {
						m_frontLeft.getPosition(),
						m_frontRight.getPosition(),
						m_rearLeft.getPosition(),
						m_rearRight.getPosition()
				});
	}

	@Override
	public void periodic() {
		SwerveModulePosition[] positions = new SwerveModulePosition[] {
				m_frontLeft.getPosition(),
				m_frontRight.getPosition(),
				m_rearLeft.getPosition(),
				m_rearRight.getPosition()
		};

		Rotation2d rotation = Rotation2d.fromDegrees(getGyroAngle());
		m_odometry.update(rotation, positions);
		poseEstimator.update(rotation, positions);
	}

	@Override
	public void simulationPeriodic() {
		m_frontLeft.simulationPeriodic();
		m_frontRight.simulationPeriodic();
		m_rearLeft.simulationPeriodic();
		m_rearRight.simulationPeriodic();

		double omegaRadPerSec = m_lastChassisSpeeds.omegaRadiansPerSecond;
		m_simGyroAngle += Math.toDegrees(omegaRadPerSec * .02);

		while (m_simGyroAngle > 180)
			m_simGyroAngle -= 360;
		while (m_simGyroAngle < -180)
			m_simGyroAngle += 360;
	}

	private double getGyroAngle() {
		if (RobotBase.isSimulation()) {
			return m_simGyroAngle;
		} else {
			return -m_gyro.getYaw();
		}
	}

	public void log() {
		Pose2d pose = getPose();
		SmartDashboard.putNumber("Robot X (m)", pose.getX());
		SmartDashboard.putNumber("Robot Y (m)", pose.getY());
		SmartDashboard.putNumber("Robot Heading (deg)", getHeading());

		SmartDashboard.putNumberArray("Robot Pose", new double[] {
				pose.getX(),
				pose.getY(),
				pose.getRotation().getRadians()
		});

		SmartDashboard.putNumberArray("Swerve States", new double[] {
				m_frontLeft.getState().angle.getRadians(),
				m_frontLeft.getState().speedMetersPerSecond,
				m_frontRight.getState().angle.getRadians(),
				m_frontRight.getState().speedMetersPerSecond,
				m_rearLeft.getState().angle.getRadians(),
				m_rearLeft.getState().speedMetersPerSecond,
				m_rearRight.getState().angle.getRadians(),
				m_rearRight.getState().speedMetersPerSecond
		});

		SmartDashboard.putNumber("Front Left Module Angle (deg)", m_frontLeft.getState().angle.getDegrees());
		SmartDashboard.putNumber("Front Left Module Speed (m/s)", m_frontLeft.getState().speedMetersPerSecond);
		SmartDashboard.putNumber("Front Right Module Angle (deg)", m_frontRight.getState().angle.getDegrees());
		SmartDashboard.putNumber("Front Right Module Speed (m/s)", m_frontRight.getState().speedMetersPerSecond);
		SmartDashboard.putNumber("Back Left Module Angle (deg)", m_rearLeft.getState().angle.getDegrees());
		SmartDashboard.putNumber("Back Left Module Speed (m/s)", m_rearLeft.getState().speedMetersPerSecond);
		SmartDashboard.putNumber("Back Right Module Angle (deg)", m_rearRight.getState().angle.getDegrees());
		SmartDashboard.putNumber("Back Right Module Speed (m/s)", m_rearRight.getState().speedMetersPerSecond);

		SmartDashboard.putNumber("Gyro Yaw (deg)", getGyroAngle());
		SmartDashboard.putNumber("Gyro Pitch", m_gyro.getPitch());
		SmartDashboard.putNumber("Gyro Roll", m_gyro.getRoll());
	}

	public double getHeading() {
		return Rotation2d.fromDegrees(getGyroAngle()).getDegrees();
	}

	public Pose2d getPose() {
		return poseEstimator.getEstimatedPosition();
	}

	public void resetOdometry(Pose2d pose) {
		SwerveModulePosition[] positions = new SwerveModulePosition[] {
				m_frontLeft.getPosition(),
				m_frontRight.getPosition(),
				m_rearLeft.getPosition(),
				m_rearRight.getPosition()
		};
		Rotation2d rotation = Rotation2d.fromDegrees(m_gyro.getYaw());

		m_odometry.resetPosition(rotation, positions, pose);
		poseEstimator.resetPosition(rotation, positions, pose);

		if (RobotBase.isSimulation()) {
			m_simGyroAngle = pose.getRotation().getDegrees();
		}
	}

	public void addVisionMeasurement(Pose2d visionMeasurement, double timestampSeconds) {
		poseEstimator.addVisionMeasurement(visionMeasurement, timestampSeconds);
	}

	public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
		double xSpeedDelivered = xSpeed * DriveConstants.kMaxSpeedMetersPerSecond;
		double ySpeedDelivered = ySpeed * DriveConstants.kMaxSpeedMetersPerSecond;
		double rotDelivered = rot * DriveConstants.kMaxAngularSpeed;

		if (fieldRelative) {
			m_lastChassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
					xSpeedDelivered, ySpeedDelivered, rotDelivered,
					Rotation2d.fromDegrees(getGyroAngle()));
		} else {
			m_lastChassisSpeeds = new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered);
		}

		var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(m_lastChassisSpeeds);
		SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);

		m_frontLeft.setDesiredState(swerveModuleStates[0]);
		m_frontRight.setDesiredState(swerveModuleStates[1]);
		m_rearLeft.setDesiredState(swerveModuleStates[2]);
		m_rearRight.setDesiredState(swerveModuleStates[3]);
	}

	public void setX() {
		m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
		m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
		m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
		m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
	}

	public void setModuleStates(SwerveModuleState[] desiredStates) {
		SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
		m_frontLeft.setDesiredState(desiredStates[0]);
		m_frontRight.setDesiredState(desiredStates[1]);
		m_rearLeft.setDesiredState(desiredStates[2]);
		m_rearRight.setDesiredState(desiredStates[3]);
	}

	public void resetEncoders() {
		m_frontLeft.resetEncoders();
		m_rearLeft.resetEncoders();
		m_frontRight.resetEncoders();
		m_rearRight.resetEncoders();
	}

	public void zeroHeading() {
		if (RobotBase.isSimulation()) {
			m_simGyroAngle = 0;
		} else {
			m_gyro.reset();
		}
	}

	public void stopModules() {
		m_lastChassisSpeeds = new ChassisSpeeds();
		m_frontLeft.setDesiredState(new SwerveModuleState(0, m_frontLeft.getState().angle));
		m_frontRight.setDesiredState(new SwerveModuleState(0, m_frontRight.getState().angle));
		m_rearLeft.setDesiredState(new SwerveModuleState(0, m_rearLeft.getState().angle));
		m_rearRight.setDesiredState(new SwerveModuleState(0, m_rearRight.getState().angle));
	}

	public void followTrajectory(SwerveSample sample) {
		SmartDashboard.putNumberArray("Choreo Target Pose", new double[] {
				sample.x,
				sample.y,
				sample.heading
		});

		Pose2d pose = getPose();

		ChassisSpeeds speeds = new ChassisSpeeds(
				sample.vx + xController.calculate(pose.getX(), sample.x),
				sample.vy + yController.calculate(pose.getY(), sample.y),
				sample.omega + headingController.calculate(pose.getRotation().getRadians(), sample.heading));

		var chassisField = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, pose.getRotation());
		m_lastChassisSpeeds = chassisField;

		var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisField);
		SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);

		m_frontLeft.setDesiredState(swerveModuleStates[0]);
		m_frontRight.setDesiredState(swerveModuleStates[1]);
		m_rearLeft.setDesiredState(swerveModuleStates[2]);
		m_rearRight.setDesiredState(swerveModuleStates[3]);

		SmartDashboard.putNumber("Choreo/TargetOmegaRadPerSec", sample.omega);
	}

	public double getTurnRate() {
		return m_gyro.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
	}
}