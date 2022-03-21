package frc.team5973.robot.rapidreact;

import java.util.Map;

// CAMERA IMPORTS
import edu.wpi.first.cameraserver.CameraServer;
// import edu.wpi.first.cscore.CvSink;
// import edu.wpi.first.cscore.CvSource;
// import edu.wpi.first.cscore.MjpegServer;
import edu.wpi.first.cscore.UsbCamera;
// import org.opencv.core.Mat;
// import org.opencv.core.Point;
// import org.opencv.core.Scalar;
// import org.opencv.imgproc.Imgproc;

import edu.wpi.first.cscore.VideoMode.PixelFormat;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.team5973.robot.RobotBase;
import frc.team5973.robot.XinputController;
import frc.team5973.robot.rapidreact.Intake.IntakeInput;
import frc.team5973.robot.rapidreact.Shooter.ShooterInput;
import frc.team5973.robot.rapidreact.Shooter.ShooterAxis;
import frc.team5973.robot.rapidreact.commands.ballCollectionCommands.CenterBallCommand;
import frc.team5973.robot.rapidreact.commands.ballCollectionCommands.CollectBallCommand;
import frc.team5973.robot.rapidreact.commands.defaultCommands.IntakeDefaultCommand;
import frc.team5973.robot.rapidreact.commands.defaultCommands.ShooterDefaultCommand;
import frc.team5973.robot.rapidreact.commands.defaultCommands.SwerveDriveDefaultCommand;
import frc.team5973.robot.subsystems.Limelight;
import frc.team5973.robot.subsystems.SwerveDrive;
import frc.team5973.robot.subsystems.SwerveDrive.Axis;
import frc.team5973.robot.subsystems.SwerveDrive.DriveMode;

public class Robot extends RobotBase {
	
	private DetectionData detectionData;
	private Limelight limelight;
	private SwerveDrive drive;
	private Shooter shooter;
	private Intake intake;

	private Command autonomousCommand;
	private Auto auto;

	private int driverPort  = 0;
	private int copilotPort = 1;

	private final double SPEED_NORMAL  = 0.9;
	private final double SPEED_SAFE    = 0.3;
	
	private final double DEADBAND_LOW  = 0.06;
	private final double DEADBAND_HIGH = 1;

	private XinputController driver  = new XinputController(driverPort);
	private XinputController copilot = new XinputController(copilotPort);

	UsbCamera camera1;
	UsbCamera camera2;
	NetworkTableEntry cameraSelection;

	public Robot() {
		super("Rapidreact");

		camera1 = CameraServer.startAutomaticCapture(0);
		camera2 = CameraServer.startAutomaticCapture(1);
	
		cameraSelection = NetworkTableInstance.getDefault().getTable("").getEntry("CameraSelection");

		/*
		new Thread(() -> {
			// Creates UsbCamera and MjpegServer [1] and connects them
			UsbCamera usbCamera0 = CameraServer.startAutomaticCapture(0);
			UsbCamera usbCamera1 = CameraServer.startAutomaticCapture(1);
			MjpegServer mjpegServer0 = new MjpegServer("serve_USB Camera 0", 1181);
			MjpegServer mjpegServer1 = new MjpegServer("serve_USB Camera 1", 1181);
			mjpegServer0.setSource(usbCamera0);
			mjpegServer1.setSource(usbCamera1);

			// Creates the CvSink and connects it to the UsbCamera
			CvSink cvSink0 = new CvSink("opencv_USB Camera 0");
			CvSink cvSink1 = new CvSink("opencv_USB Camera 1");
			cvSink0.setSource(usbCamera0);
			cvSink1.setSource(usbCamera1);

			// Creates the CvSource and MjpegServer [2] and connects them
			CvSource outputStream0 = new CvSource("Blur0", PixelFormat.kMJPEG, 640, 480, 30);
			CvSource outputStream1 = new CvSource("Blur1", PixelFormat.kMJPEG, 640, 480, 30);
			MjpegServer mjpegServ0 = new MjpegServer("serve_Blur0", 1182);
			MjpegServer mjpegServ1 = new MjpegServer("serve_Blur1", 1182);
			mjpegServ0.setSource(outputStream0);
			mjpegServ1.setSource(outputStream1);

			Mat source0 = new Mat();
      		Mat output0 = new Mat();
			Mat source1 = new Mat();
      		Mat output1 = new Mat();

			while(!Thread.interrupted()) {
				if (cvSink0.grabFrame(source0) == 0) {
				  continue;
				}
				Imgproc.cvtColor(source0, output0, Imgproc.COLOR_BGR2GRAY);
				outputStream0.putFrame(output0);
				Imgproc.cvtColor(source1, output1, Imgproc.COLOR_BGR2GRAY);
				outputStream0.putFrame(output1);
			}


		}).start();
		*/

        port("shooterWheel",  12);  
		port("shooterOutake", 8); 
		
		port("intakeWheels", 10);
		port("leftArm", 0);
		port("rightArm", 1);
				
		port("frontRightSpeedMotor", 3);
        port("frontLeftSpeedMotor",  0);
        port("backRightSpeedMotor",  2);
        port("backLeftSpeedMotor",   1);
        
        port("frontRightAngleMotor", 7);
        port("frontLeftSAngleMotor", 4);
        port("backRightAngleMotor",  6);
        port("backLeftAngleMotor",   5);

		drive = new SwerveDrive(this);
		
		shooter = new Shooter(this);
		intake  = new Intake(this);

		detectionData = new DetectionData(this);
		limelight     = new Limelight(this);
	
		drive.setDefaultCommand(new SwerveDriveDefaultCommand(drive, limelight, 
																	 detectionData, DEADBAND_HIGH,
																	 				DEADBAND_LOW,
																					SPEED_NORMAL, 
																					SPEED_SAFE, 
																					Map.of(
			Axis.FORWARD, () -> driver.getLeftY(),
			Axis.STRAFE,  () -> driver.getLeftX(),
			Axis.TURN,    () -> driver.getRightX()),
			Map.of(
			DriveMode.SAFEMMODE, () -> driver.getLeftStickButton() && driver.getRightStickButton(),
			DriveMode.FIELDMODE, () -> copilot.getBButton(),
			DriveMode.GOALMODE,  () -> driver.getAButton(),
			DriveMode.BALLMODE,  () -> copilot.getYButton(),
			DriveMode.ZERO_GYRO, () -> copilot.getXButton()
			)));

		shooter.setDefaultCommand(new ShooterDefaultCommand(shooter, limelight, Map.of(
			ShooterInput.BUTTON, () -> driver.getBButton()
		), Map.of(
			ShooterAxis.INTERNAL_WHEEL, () -> copilot.getLeftY()
		)));

		intake.setDefaultCommand(new IntakeDefaultCommand(intake, Map.of(
			IntakeInput.EXTEND, () -> copilot.getLeftBumper(),
			IntakeInput.COLLECT, () -> copilot.getRightBumper(),
			IntakeInput.COLLECT_CEDRIC, () -> copilot.getAButton()
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
