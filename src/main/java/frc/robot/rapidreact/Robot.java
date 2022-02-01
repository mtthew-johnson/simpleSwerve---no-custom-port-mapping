package frc.robot.rapidreact;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.robot.RobotBase;
import frc.robot.Xinput;
import frc.robot.subsystems.SwerveDrive;

public class Robot extends RobotBase {
	
	public SwerveDrive drive;
	private Command autonomousCommand;

	public Robot() {
		super("Rapidreact");

		int mainJoystick = 0;

		JoystickButton joystickStart = new JoystickButton(new Joystick(mainJoystick), Xinput.Start);

        port("frontRightSpeedMotor", 5);
        port("frontLeftSpeedMotor", 1);
        port("backRightSpeedMotor", 6);
        port("backLeftSpeedMotor", 2);
        
        port("frontRightAngleMotor", 4);
        port("frontLeftSAngleMotor", 0);
        port("backRightAngleMotor", 7);
        port("backLeftAngleMotor", 3);

		axis("forward", mainJoystick, Xinput.LeftStickY);
		axis("strafe", mainJoystick, Xinput.LeftStickX);
		axis("rotate", mainJoystick, Xinput.RightStickX);

		joystickStart.whenPressed(new InstantCommand(() -> {

			drive.fieldOrientedMode = !drive.fieldOrientedMode;
			drive.gyro.reset();

		}));

		button("driveModeToggle", mainJoystick, Xinput.Start);
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
