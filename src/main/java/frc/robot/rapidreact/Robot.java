package frc.robot.rapidreact;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.RobotBase;
import frc.robot.Xinput;
import frc.robot.XinputController;
import frc.robot.subsystems.SwerveDrive;
public class Robot extends RobotBase {
	
	public SwerveDrive drive;
	XinputController mainJoystick = new XinputController(0);

	private Command autonomousCommand;

	public Robot() {
		super("Rapidreact");

		int mainJoystick = 0;

        port("serve/frontRightSpeedMotor", 5);
        port("serve/frontLeftSpeedMotor", 1);
        port("serve/backRightSpeedMotor", 6);
        port("serve/backLeftSpeedMotor", 2);
        
        port("serve/frontRightAngleMotor", 4);
        port("serve/frontLeftSAngleMotor", 0);
        port("serve/backRightAngleMotor", 7);
        port("serve/backLeftAngleMotor", 3);

		axis("forward", mainJoystick, Xinput.LeftStickY);
		axis("strafe", mainJoystick, Xinput.LeftStickX);
		axis("rotate", mainJoystick, Xinput.RightStickX);

	
		button("safeModeToggle", () -> button(0, Xinput.LeftStickIn) && button(0, Xinput.RightStickIn));

		drive = addSubsystem(SwerveDrive::new);

	}

	@Override
	public void teleopInit() {
		if (autonomousCommand != null)
			autonomousCommand.cancel();
	}

	@Override
	public void teleopPeriodic() {
		CommandScheduler.getInstance().run();
	}

	@Override
	public void testPeriodic() {
		CommandScheduler.getInstance().run();
		
	}

	@Override
	public void disabledInit() {
		
	}

	@Override
	public void autonomousInit() {

	}

	@Override
	public void autonomousPeriodic() {
		CommandScheduler.getInstance().run();
	}

}
