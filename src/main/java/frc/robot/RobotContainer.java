// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.Vision;

public class RobotContainer {
	// The robot's subsystems
	private final DriveSubsystem m_drive = new DriveSubsystem();
	private final IntakeSubsystem m_intake = new IntakeSubsystem();
	private final Vision m_vision = new Vision(m_drive::addVisionMeasurement);
	private final ShooterSubsystem m_shooter = new ShooterSubsystem();

	private final Field2d m_field = new Field2d();

	// AUTO CHOOSER
	private final SendableChooser<Command> autoChooser = new SendableChooser<>();

	// The driver's controller
	private final XboxController driverController = new XboxController(0);

	// Field Relativity
	private boolean fieldRelative = true;

	/**
	 * The container for the robot. Contains subsystems, OI devices, and commands.
	 */
	public RobotContainer() {
		m_shooter.useVision(m_vision);
		// CONFIGURE AUTO CHOOSER
		autoChooser.setDefaultOption("Go Back Time", Autos.goBackTime(m_drive, m_shooter));
		autoChooser.addOption("followChoreo", Autos.followChoreoTrajectory(m_drive, "Driveback_1m"));
		autoChooser.addOption("Stationary Shoot", Autos.stationaryShoot(m_shooter));
		autoChooser.addOption("Do Nothing", Autos.doNothing());

		// Configure the button bindings
		configureButtonBindings(

		);

		// Configure default commands
		m_drive.setDefaultCommand(
				// The left stick controls translation of the robot.
				// Turning is controlled by the X axis of the right stick.
				// NOTE: Using robot-relative drive (fieldRelative = false) for normal teleop
				new RunCommand(
						() -> {
							double xSpeed = -MathUtil.applyDeadband(driverController.getLeftY(),
									OIConstants.kDriveDeadband);
							double ySpeed = -MathUtil.applyDeadband(driverController.getLeftX(),
									OIConstants.kDriveDeadband);
							double rot = -MathUtil.applyDeadband(driverController.getRightX(),
									OIConstants.kDriveDeadband);

							// Robot-relative drive: false = robot-relative, true = field-relative
							SmartDashboard.putString("Drive Mode", fieldRelative ? "Field-Relative" : "Robot-Relative");
							m_drive.drive(xSpeed, ySpeed, rot, fieldRelative);

						}, m_drive));

		// AUTO CHOOSER
		SmartDashboard.putData("[Smart Dashboard] autoChooser", autoChooser);

		if (RobotBase.isSimulation()) {
			configureFuelSim();
		}
	}

	private void configureFuelSim() {
	}

	/**
	 * Use this method to define your button->command mappings. Buttons can be
	 * created by
	 * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
	 * subclasses ({@link
	 * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
	 * passing it to a
	 * {@link JoystickButton}.
	 */
	private void configureButtonBindings() {
		SmartDashboard.putData("Field", m_field);
		SmartDashboard.putNumber("Match Time", DriverStation.getMatchTime());

		new Trigger(() -> driverController.getLeftTriggerAxis() > 0.1)
				.whileTrue(m_vision.alignToTag(m_drive, driverController,
						Alliance.Red == DriverStation.getAlliance().orElse(Alliance.Red)
								? frc.robot.Constants.Vision.redTags
								: frc.robot.Constants.Vision.blueTags));

		new Trigger(() -> driverController.getRightTriggerAxis() > 0.1)
				.whileTrue(m_shooter.fullShootVisionCommand())
				.onFalse(m_shooter.fullStopCommand());

		new JoystickButton(driverController, XboxController.Button.kLeftBumper.value)
				.onTrue(m_intake.runIntakeCommand(3000))
				.onFalse(m_intake.stopIntakeCommand());

		new JoystickButton(driverController, XboxController.Button.kRightBumper.value)
				.whileTrue(m_shooter.fullShootCommand(3000, 4000))
				.onFalse(m_shooter.fullStopCommand());

		new Trigger(() -> driverController.getPOV() == 0)
				.onTrue(m_intake.runUpCommand())
				.onFalse(m_intake.runStopCommand());

		new Trigger(() -> driverController.getPOV() == 180)
				.onTrue(m_intake.runDownCommand())
				.onFalse(m_intake.runStopCommand());

		new JoystickButton(driverController, XboxController.Button.kY.value)
				.onTrue(new InstantCommand(() -> m_drive.zeroHeading()));

		new JoystickButton(driverController, XboxController.Button.kStart.value)
				.whileTrue(new RunCommand(() -> m_drive.setX(), m_drive));

		// A Button- Allign to Tag 25
		new JoystickButton(driverController, XboxController.Button.kX.value)
				.whileTrue(m_vision.alignToTag(m_drive, driverController,
						Alliance.Red == DriverStation.getAlliance().orElse(Alliance.Red)
								? frc.robot.Constants.Vision.redTags
								: frc.robot.Constants.Vision.blueTags));
	}

	public void periodic() {
		m_field.setRobotPose(m_drive.getPose());
	}

	/**
	 * Use this to pass the autonomous command to the main {@link Robot} class.
	 *
	 * @return the command to run in autonomous
	 */
	public DriveSubsystem getM_drive() {
		return m_drive;
	}

	public Vision getM_vision() {
		return m_vision;
	}

	public Command getAutonomousCommand() {
		return autoChooser.getSelected();
	}
}
