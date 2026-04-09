package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants.IntakeSubsystemConstants;

public class IntakeSubsystem extends SubsystemBase {
	// Initialize intake SPARK. We will use open loop control for this
	private final SparkMax IntakeMotor = new SparkMax(IntakeSubsystemConstants.kIntakeMotorCanId, MotorType.kBrushless);

	// Initialize intake SPARK. We will use open loop control for this
	private final SparkMax PivotMotor = new SparkMax(IntakeSubsystemConstants.kPivotMotorCanId, MotorType.kBrushless);

	public IntakeSubsystem() {

		IntakeMotor.configure(Configs.Intake.INTAKE_CONFIG, ResetMode.kResetSafeParameters,
				PersistMode.kPersistParameters);

		PivotMotor.configure(Configs.Intake.PIVOT_CONFIG, ResetMode.kResetSafeParameters,
				PersistMode.kPersistParameters);

	}

	public boolean m_up = true;

	// Set the intake motor power in the range of [-1, 1]
	private void setIntakePower(double power) {
		IntakeMotor.set(power);
	}

	private void setPivotPower(double power) {
		PivotMotor.set(power);
	}

	// Command to run the intake and pivot motors. When the command is interrupted,
	// ex is if the button is released, the motors will stop

	public Command runIntakeCommand() {
		return this.startEnd(() -> setIntakePower(-.6), () -> setIntakePower(0));
	}

	public Command stopIntakeCommand() {
		return new InstantCommand(() -> {
			setIntakePower(0);
		}, this);
	}

	public Command runUpCommand() {
		return new InstantCommand(() -> {
			setPivotPower(-.2);
			m_up = true;
		}, this);
	}

	public Command runDownCommand() {
		return new InstantCommand(() -> {
			setPivotPower(.2);
			m_up = false;
		}, this);
	}

	public Command runStopCommand() {
		return new InstantCommand(() -> {
			setPivotPower(0);
		}, this);
	}

	public Command fullRunUpCommand() {
		return Commands.sequence(
				runUpCommand(),
				Commands.waitSeconds(.25),
				runStopCommand());
	}

	public Command fullRunDownCommand() {
		return Commands.sequence(
				runDownCommand(),
				Commands.waitSeconds(.25),
				runStopCommand());
	}

	public Command toggleArmCommand() {
		return new InstantCommand(() -> {
			if (m_up) {
				setPivotPower(.2);
				m_up = false;
			} else {
				setPivotPower(-.2);
				m_up = true;
			}
		}, this);
	}
}
