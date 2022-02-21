package frc.robot.rapidreact.intake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.motorcontrol.Jaguar;

import frc.robot.RobotBase;
import frc.robot.subsystems.SubsystemBase;

public class Intake extends SubsystemBase {

	private final Timer timer = new Timer();

	private final double INTAKE_SPEED = 1;

	private Jaguar intake;
	private Jaguar rollerWheels;

	public enum IntakeInput {BUTTON}

	public Intake(RobotBase robot) {

		super(robot);

        configMotors();
	}

    private void configMotors() {
        
        intake       = new Jaguar(2);//new Jaguar(configInt("intake"));
		rollerWheels = new Jaguar(3);//new Jaguar(configInt("rollerWheels"));

		addChild("intake",       intake);
		addChild("rollerWheels", rollerWheels);

        intake.setInverted(false);
		rollerWheels.setInverted(true);
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