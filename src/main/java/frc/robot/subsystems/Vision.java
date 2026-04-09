package frc.robot.subsystems;

import java.util.Optional;
import java.util.function.BiConsumer;

import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;

public class Vision extends SubsystemBase {

	public record AlignTarget(int id, double yaw, String cameraName) {
	}

	private final PhotonCamera frontLeftCamera;
	private final PhotonCamera frontRightCamera;
	private final PhotonPoseEstimator frontLeftEstimator;
	private final PhotonPoseEstimator frontRightEstimator;
	private VisionSystemSim visionSim;
	private final BiConsumer<Pose2d, Double> poseConsumer;

	public Vision(BiConsumer<Pose2d, Double> poseConsumer) {
		this.poseConsumer = poseConsumer;

		frontLeftCamera = new PhotonCamera(Constants.Vision.kFrontLeftCameraName);
		frontRightCamera = new PhotonCamera(Constants.Vision.kFrontRightCameraName);

		frontLeftEstimator = new PhotonPoseEstimator(
				Constants.Vision.kTagLayout,
				PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
				Constants.Vision.kRobotToFrontLeftCamera);
		frontLeftEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

		frontRightEstimator = new PhotonPoseEstimator(
				Constants.Vision.kTagLayout,
				PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
				Constants.Vision.kRobotToFrontRightCamera);
		frontRightEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

		if (Robot.isSimulation()) {
			visionSim = new VisionSystemSim("main");
			visionSim.addAprilTags(Constants.Vision.kTagLayout);

			var cameraProp = new SimCameraProperties();
			cameraProp.setCalibration(960, 720, Rotation2d.fromDegrees(90));
			cameraProp.setCalibError(.35, .10);
			cameraProp.setFPS(60);
			cameraProp.setAvgLatencyMs(50);
			cameraProp.setLatencyStdDevMs(15);

			var frontLeftCamSim = new PhotonCameraSim(frontLeftCamera, cameraProp);
			visionSim.addCamera(frontLeftCamSim, Constants.Vision.kRobotToFrontLeftCamera);
			frontLeftCamSim.enableDrawWireframe(true);

			var frontRightCamSim = new PhotonCameraSim(frontRightCamera, cameraProp);
			visionSim.addCamera(frontRightCamSim, Constants.Vision.kRobotToFrontRightCamera);
			frontRightCamSim.enableDrawWireframe(true);
		}

		CameraServer.addCamera(new HttpCamera(
				Constants.Vision.kFrontLeftCameraName,
				"http://10.94.91.11:1182/stream.mjpg"));
	}

	@Override
	public void periodic() {
		processCameraResult(frontLeftCamera, frontLeftEstimator, "FL");
		processCameraResult(frontRightCamera, frontRightEstimator, "FR");
	}

	private void processCameraResult(PhotonCamera camera, PhotonPoseEstimator estimator, String prefix) {
		PhotonPipelineResult result = camera.getLatestResult();
		SmartDashboard.putBoolean("Vision/" + prefix + "/Connected", camera.isConnected());
		SmartDashboard.putBoolean("Vision/" + prefix + "/Has Targets", result.hasTargets());

		if (result.hasTargets()) {
			SmartDashboard.putNumber("Vision/" + prefix + "/Target Count", result.getTargets().size());
			SmartDashboard.putNumber("Vision/" + prefix + "/Best Tag ID", result.getBestTarget().getFiducialId());
		}

		if (!result.hasTargets())
			return;

		Optional<EstimatedRobotPose> estimatedPose = estimator.update(result);
		if (estimatedPose.isEmpty())
			return;

		EstimatedRobotPose pose = estimatedPose.get();
		Pose2d pose2d = pose.estimatedPose.toPose2d();

		SmartDashboard.putNumber("Vision/" + prefix + "/Pose X", pose2d.getX());
		SmartDashboard.putNumber("Vision/" + prefix + "/Pose Y", pose2d.getY());
		SmartDashboard.putNumber("Vision/" + prefix + "/Timestamp", pose.timestampSeconds);

		Matrix<N3, N1> stdDevs = computeStdDevs(pose, estimator);
		SmartDashboard.putString("Vision/" + prefix + "/Std Devs",
				String.format("[%.2f, %.2f, %.2f]",
						stdDevs.get(0, 0), stdDevs.get(1, 0), stdDevs.get(2, 0)));

		poseConsumer.accept(pose2d, pose.timestampSeconds);
	}

	private Matrix<N3, N1> computeStdDevs(EstimatedRobotPose estimatedPose, PhotonPoseEstimator estimator) {
		Matrix<N3, N1> estStdDevs = Constants.Vision.kSingleTagStdDevs;
		int numTags = estimatedPose.targetsUsed.size();
		double avgDist = 0;

		for (var target : estimatedPose.targetsUsed) {
			var tagPose = estimator.getFieldTags().getTagPose(target.getFiducialId());
			if (tagPose.isPresent()) {
				avgDist += tagPose.get().toPose2d().getTranslation()
						.getDistance(estimatedPose.estimatedPose.toPose2d().getTranslation());
			}
		}

		if (numTags > 0)
			avgDist /= numTags;

		if (numTags > 1) {
			estStdDevs = Constants.Vision.kMultiTagStdDevs;
		}

		if (numTags == 1 && avgDist > 4) {
			estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
		} else {
			estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
		}

		return estStdDevs;
	}

	// -------------------------------------------------------------------------
	// Align to tag
	// -------------------------------------------------------------------------
	public Command alignToTag(DriveSubsystem drivetrain, int... tagIds) {
		PIDController yaw = new PIDController(
				Constants.Vision.kAlignP,
				Constants.Vision.kAlignI,
				Constants.Vision.kAlignD);
		yaw.setTolerance(Constants.Vision.kAlignToleranceDeg);

		return this.runEnd(
				() -> findAlignTarget(tagIds).ifPresentOrElse(
						t -> drivetrain.drive(0, 0, MathUtil.clamp(
								yaw.calculate(t.yaw(), 0) / Constants.DriveConstants.kMaxAngularSpeed,
								-0.5, 0.5), true),
						drivetrain::stopModules),
				() -> {
					drivetrain.stopModules();
					yaw.close();
				}).withName("AlignToTag");
	}

	private Optional<AlignTarget> findAlignTarget(int... tagIds) {
		for (PhotonCamera cam : new PhotonCamera[] { frontLeftCamera, frontRightCamera }) {
			PhotonPipelineResult result = cam.getLatestResult();
			if (!result.hasTargets())
				continue;
			for (PhotonTrackedTarget t : result.getTargets())
				for (int id : tagIds)
					if (t.getFiducialId() == id)
						return Optional.of(new AlignTarget(id, t.getYaw(), cam.getName()));
		}
		return Optional.empty();
	}

	// -------------------------------------------------------------------------
	// Misc
	// -------------------------------------------------------------------------
	public double getHubDistance() {
		for (PhotonCamera camera : new PhotonCamera[] { frontLeftCamera, frontRightCamera }) {
			PhotonPipelineResult result = camera.getLatestResult();
			if (!result.hasTargets())
				continue;
			for (var target : result.getTargets()) {
				int id = target.getFiducialId();
				if (id == 10 || id == 26 || id == 8 || id == 11 || id == 18 || id == 21) {
					double distance = target.getBestCameraToTarget().getTranslation().getNorm();
					SmartDashboard.putNumber("Shooter/Distance (m)", distance);
					return distance;
				}
			}
		}
		SmartDashboard.putNumber("Shooter/Distance (m)", -1);
		return -1;
	}

	public PhotonCamera getFrontLeftCamera() {
		return frontLeftCamera;
	}

	public PhotonCamera getFrontRightCamera() {
		return frontRightCamera;
	}

	public void simulationPeriodic(Pose2d robotSimPose) {
		if (visionSim != null)
			visionSim.update(robotSimPose);
	}

	public void resetSimPose(Pose2d pose) {
		if (Robot.isSimulation() && visionSim != null)
			visionSim.resetRobotPose(pose);
	}
}