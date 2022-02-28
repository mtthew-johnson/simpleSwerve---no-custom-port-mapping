package frc.robot.rapidreact;

import java.util.Map;

import edu.wpi.first.wpilibj.ADIS16448_IMU;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.ADIS16470_IMU.CalibrationTime;
import edu.wpi.first.wpilibj.ADIS16470_IMU.IMUAxis;
import edu.wpi.first.wpilibj.SPI.Port;
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
	private ADIS16470_IMU gyro;

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

        port("shooterWheel",  9);  
		port("shooterOutake", 8); 
		port("intake", 10);       
		
		port("frontRightSpeedMotor", 3);
        port("frontLeftSpeedMotor",  0);
        port("backRightSpeedMotor",  2);
        port("backLeftSpeedMotor",   1);
        
        port("frontRightAngleMotor", 7);
        port("frontLeftSAngleMotor", 4);
        port("backRightAngleMotor",  6);
        port("backLeftAngleMotor",   5);
		
		//button("safeModeToggle", () -> button(0, Xinput.LeftStickIn) && button(0, Xinput.RightStickIn));

		//button("shoot",      driverPort, Xinput.A);
		//button("intakeBall", driverPort, Xinput.B);

		drive = new SwerveDrive(this);
		
		shooter = new Shooter(this);
		intake  = new Intake(this);

		detectionData = new DetectionData(this);
		limelight     = new Limelight(this);

		gyro = new ADIS16470_IMU();
	
		
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
