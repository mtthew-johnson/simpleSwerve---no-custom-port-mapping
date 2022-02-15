package frc.robot.rapidreact;

import java.util.Map;

import edu.wpi.first.math.MathUtil;
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
import frc.robot.rapidreact.intake.Intake;
import frc.robot.rapidreact.intake.IntakeDefaultCommand;
import frc.robot.rapidreact.intake.IntakeBase.IntakeInput;
import frc.robot.rapidreact.shooter.Shooter;
import frc.robot.rapidreact.shooter.ShooterDefaultCommand;
import frc.robot.rapidreact.shooter.ShooterBase.ShooterInput;
import frc.robot.subsystems.Gyro;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.SwerveDriveDefaultCommand;
import frc.robot.subsystems.SwerveBase.Input;

public class Robot extends RobotBase {
	
	public SwerveDrive drive;
	public Shooter shooter;
	public Intake intake;
	public Gyro gyro;

	private Command autonomousCommand;
	private Auto auto;

	private int driverPort = 0;
	private double deadband = 0.2;

	private boolean driveMode = false;

	private XinputController driver = new XinputController(driverPort);

	public Robot() {
		super("Rapidreact");

		JoystickButton joystickStart = new JoystickButton(new Joystick(driverPort), Xinput.Start);

        port("shooterWheel", 1);  
		port("shooterOutake", 0); 
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

		// axis("forward", mainJoystickPort, Xinput.LeftStickY);
		// axis("strafe", mainJoystickPort, Xinput.LeftStickX);
		// axis("rotate", mainJoystickPort, Xinput.RightStickX);
		
		//button("safeModeToggle", () -> button(0, Xinput.LeftStickIn) && button(0, Xinput.RightStickIn));

		//button("shoot",      driverPort, Xinput.A);
		//button("intakeBall", driverPort, Xinput.B);

		drive = new SwerveDrive(this);
		
		shooter = new Shooter(this);
		intake  = new Intake(this);
		
		drive.setDefaultCommand(new SwerveDriveDefaultCommand(drive, gyro, Map.of(
			Input.FORWARD, () ->  MathUtil.applyDeadband(driver.getLeftY(),  deadband),
			Input.STRAFE,  () -> -MathUtil.applyDeadband(driver.getLeftX(),  deadband),
			Input.TURN,    () -> -MathUtil.applyDeadband(driver.getRightX(), deadband)
		)));

		shooter.setDefaultCommand(new ShooterDefaultCommand(shooter, Map.of(
			ShooterInput.BUTTON, () -> driver.getAButton()
		)));

		intake.setDefaultCommand(new IntakeDefaultCommand(intake, Map.of(
			IntakeInput.BUTTON, () -> driver.getBButton()
		)));
		
		// joystickStart.whenPressed(new InstantCommand(() -> {
		// 	driveMode = !driveMode;
		// 	drive.setIsFieldOriented(driveMode);
		// 	gyro.reset();
		// }));
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
