package frc.robot.rapidreact;

import static edu.wpi.first.wpilibj.XboxController.Button.*;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.robot.RobotBase;
import frc.robot.Xinput;
import frc.robot.XinputController;
import frc.robot.rapidreact.commands.ballCollectionCommands.CenterBall;
import frc.robot.rapidreact.commands.ballCollectionCommands.CollectBall;
import frc.robot.subsystems.SwerveDrive;

public class Robot extends RobotBase {
	
	public SwerveDrive drive;
	public Shooter shooter;
	public Intake intake;

	private Command autonomousCommand;
	private Auto auto;

	int mainJoystickPort = 0;

	XinputController mainJoystick = new XinputController(mainJoystickPort);

	public Robot() {
		super("Rapidreact");


		JoystickButton joystickStart = new JoystickButton(new Joystick(mainJoystickPort), Xinput.Start);

        port("shooterWheel", 0);  //TODO: the port mapping stuff doesn't work with jaguars for some reason
		port("shooterOutake", 1); 
		port("intake", 2);       
		port("rollerWheels", 3);
		
		port("frontRightSpeedMotor", 5);
        port("frontLeftSpeedMotor", 1);
        port("backRightSpeedMotor", 6);
        port("backLeftSpeedMotor", 2);
        
        port("frontRightAngleMotor", 4);
        port("frontLeftSAngleMotor", 0);
        port("backRightAngleMotor", 7);
        port("backLeftAngleMotor", 3);

		axis("forward", mainJoystickPort, Xinput.LeftStickY);
		axis("strafe", mainJoystickPort, Xinput.LeftStickX);
		axis("rotate", mainJoystickPort, Xinput.RightStickX);

		joystickStart.whenPressed(new InstantCommand(() -> {

			drive.fieldOrientedMode = !drive.fieldOrientedMode;
			drive.gyro.reset();

		}));

		button("driveModeToggle", mainJoystickPort, Xinput.Start);
		button("safeModeToggle", () -> button(0, Xinput.LeftStickIn) && button(0, Xinput.RightStickIn));

		button("shoot",      mainJoystickPort, Xinput.A);
		button("intakeBall", mainJoystickPort, Xinput.B);

		drive = new SwerveDrive(this);

		shooter = new Shooter(this);
		intake = new Intake(this);
		

		// mainJoystick.getButton(kX).whenHeld(new CenterBall(this, drive));
		// mainJoystick.getButton(kY).whenHeld(new CollectBall(this, drive, intake, 0.7));
		
		//initialize auto commands
		//auto = new Auto(drive, intake, shooter);

	}

	@Override
	public void teleopInit() {
		if (autonomousCommand != null)
			autonomousCommand.cancel();

		// if (auto != null)
		// 	auto.cancel();
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

		//auto.schedule();
		
	}

	@Override
	public void autonomousPeriodic() {
		CommandScheduler.getInstance().run();
	}

}
