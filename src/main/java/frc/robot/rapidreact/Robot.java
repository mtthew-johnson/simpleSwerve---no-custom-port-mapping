package frc.robot.rapidreact;

import java.util.Map;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import frc.robot.RobotBase;
import frc.robot.XinputController;
import frc.robot.rapidreact.commands.ballCollectionCommands.CenterBall;
import frc.robot.rapidreact.commands.ballCollectionCommands.CollectBall;
import frc.robot.rapidreact.intake.Intake;
import frc.robot.rapidreact.intake.IntakeDefaultCommand;
import frc.robot.rapidreact.intake.Intake.IntakeInput;
import frc.robot.rapidreact.shooter.Shooter;
import frc.robot.rapidreact.shooter.ShooterDefaultCommand;
import frc.robot.rapidreact.shooter.Shooter.ShooterInput;
import frc.robot.subsystems.Gyro;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.SwerveDriveDefaultCommand;
import frc.robot.subsystems.SwerveDrive.Axis;
import frc.robot.subsystems.SwerveDrive.DriveMode;

public class Robot extends RobotBase {
	
	private DetectionData detectionData;
	private Limelight limelight;
	private SwerveDrive drive;
	private Shooter shooter;
	private Intake intake;
	private Gyro gyro;

	private Command autonomousCommand;
	private Auto auto;

	private int driverPort  = 0;
	private int copilotPort = 1;

	private final double SPEED_NORMAL = 1;
	private final double SPEED_SAFE   = 0.3;
	private final double DEADBAND     = 0.1;


	private XinputController driver  = new XinputController(driverPort);
	private XinputController copilot = new XinputController(copilotPort);

	public Robot() {
		super("Rapidreact");

        port("shooterWheel",  1);  
		port("shooterOutake", 0); 
		port("intake", 2);       
		port("rollerWheels", 3);
		
		port("frontRightSpeedMotor", 5);
        port("frontLeftSpeedMotor",  1);
        port("backRightSpeedMotor",  6);
        port("backLeftSpeedMotor",   2);
        
        port("frontRightAngleMotor", 4);
        port("frontLeftSAngleMotor", 0);
        port("backRightAngleMotor",  7);
        port("backLeftAngleMotor",   3);
		
		//button("safeModeToggle", () -> button(0, Xinput.LeftStickIn) && button(0, Xinput.RightStickIn));

		//button("shoot",      driverPort, Xinput.A);
		//button("intakeBall", driverPort, Xinput.B);

		drive = new SwerveDrive(this).withGyro(gyro);
		
		shooter = new Shooter(this);
		intake  = new Intake(this);
		
		drive.setDefaultCommand(new SwerveDriveDefaultCommand(drive, limelight, 
																	 detectionData, gyro, SPEED_NORMAL, 
																					  	  SPEED_SAFE, 
																					      DEADBAND, Map.of(
			Axis.FORWARD, () -> driver.getLeftY(),
			Axis.STRAFE,  () -> driver.getLeftX(),
			Axis.TURN,    () -> driver.getRightX()
			),
			Map.of(
			DriveMode.SAFEMMODE, () -> driver.getLeftStickButton() && driver.getRightStickButton(),
			DriveMode.FIELDMODE, () -> driver.getStartButton(),
			DriveMode.GOALMODE,  () -> driver.getXButton(),
			DriveMode.BALLMODE,  () -> driver.getYButton()
			)));

		shooter.setDefaultCommand(new ShooterDefaultCommand(shooter, Map.of(
			ShooterInput.BUTTON, () -> driver.getAButton()
		)));

		intake.setDefaultCommand(new IntakeDefaultCommand(intake, Map.of(
			IntakeInput.BUTTON, () -> driver.getBButton()
		)));
	
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
