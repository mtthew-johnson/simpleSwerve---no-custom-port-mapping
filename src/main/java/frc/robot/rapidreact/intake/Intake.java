package frc.robot.rapidreact.intake;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.Timer;

import frc.robot.RobotBase;
import frc.robot.subsystems.SubsystemBase;

public class Intake extends SubsystemBase {

	private final Timer timer = new Timer();

	private final double INTAKE_SPEED = 1;

	private WPI_TalonSRX intake;

	public enum IntakeInput {BUTTON}

	public Intake(RobotBase robot) {

		super(robot);

        configMotors();
	}

    private void configMotors() {
        
        intake       = new WPI_TalonSRX(10);

		addChild("intake", intake);

        intake.setInverted(true);
    }

	public void intake() {
		intake.set(INTAKE_SPEED);
	}

	public void halt() {
		intake.set(0);
	}

	public void intakeTime(double time) {

		timer.reset();
        timer.start();

        while (!timer.hasElapsed(time)) {}
        timer.stop();

		halt();

	}
}