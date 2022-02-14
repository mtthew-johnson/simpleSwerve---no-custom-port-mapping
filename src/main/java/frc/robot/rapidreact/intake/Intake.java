package frc.robot.rapidreact.intake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.motorcontrol.Jaguar;

import frc.robot.RobotBase;

public class Intake extends IntakeBase {

	private final Timer timer = new Timer();

	private final double INTAKE_SPEED = 0.3;

	private Jaguar intake;
	private Jaguar rollerWheels;

	public Intake(RobotBase robot) {

		super(robot);

        configMotors();
	}

    private void configMotors() {
        
        intake       = new Jaguar(configInt("intake"));
		rollerWheels = new Jaguar(configInt("rollerWheels"));

		addChild("intake",       intake);
		addChild("rollerWheels", rollerWheels);

        intake.setInverted(false);
		rollerWheels.setInverted(false);
    }

	public void intake() {
		intake.set(INTAKE_SPEED);
		rollerWheels.set(INTAKE_SPEED);
	}

	public void halt() {
		intake.set(0);
		rollerWheels.set(0);
	}

	public void intakeTime(double time) {

		timer.reset();
        timer.start();

        while (!timer.hasElapsed(time)) {}
        timer.stop();

		halt();

	}
}