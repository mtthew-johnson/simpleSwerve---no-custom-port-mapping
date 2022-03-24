package frc.team5973.robot.rapidreact;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Timer;
import frc.team5973.robot.RobotBase;
import frc.team5973.robot.subsystems.SubsystemBase;

public class Intake extends SubsystemBase {

	private final Timer timer = new Timer();

	private final double INTAKE_SPEED = 0.6;

	public WPI_TalonFX intakeWheels;

	private Servo leftArm;
	private Servo rightArm;

	public enum IntakeInput {EXTEND, COLLECT, COLLECT_CEDRIC}

	public Intake(RobotBase robot) {

		super(robot);
 
        configMotors();
	}

    private void configMotors() {
        
        intakeWheels = new WPI_TalonFX(14);

		leftArm  = new Servo(port("leftArm"));
		rightArm = new Servo(1);

		addChild("intakeWheels", intakeWheels);
		addChild("leftArm",  leftArm);
		addChild("rightArm", rightArm);

        intakeWheels.setInverted(false);

		intakeWheels.setNeutralMode(NeutralMode.Coast);

		// leftArm.setSpeed(1);
		// rightArm.setSpeed(1);
    }

	public void collect() {
		intakeWheels.set(INTAKE_SPEED);
	}

	public void halt() {
		intakeWheels.set(0);
	}

	public void extend() {
		leftArm.setAngle(0);
		rightArm.setAngle(30);
	
	}

	public void retract() {
		leftArm.setAngle(30);
		rightArm.setAngle(0);

	}

	public double getIntakeSpeed() {
		return intakeWheels.getSelectedSensorVelocity();
	}

	public void intakeTime(double time) {

		timer.reset();
        timer.start();

        while (!timer.hasElapsed(time)) {}
        timer.stop();

		halt();

	}
}