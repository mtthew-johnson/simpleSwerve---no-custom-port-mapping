package frc.team5973.robot.rapidreact;

import java.util.Map;

// CAMERA IMPORTS
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.MjpegServer;
import edu.wpi.first.cscore.UsbCamera;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import edu.wpi.first.cscore.VideoMode.PixelFormat;
import edu.wpi.first.math.trajectory.Trajectory;
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
	
	private final double DEADBAND_LOW  = 0.04;
	private final double DEADBAND_HIGH = 1;

	private XinputController driver  = new XinputController(driverPort);
	private XinputController copilot = new XinputController(copilotPort);

	Thread m_visionThread; 

	public Robot() {
		super("Rapidreact");


		m_visionThread =
        new Thread(
            () -> {
              // Get the UsbCamera from CameraServer
              UsbCamera camera = CameraServer.startAutomaticCapture();
              // Set the resolution
              camera.setResolution(640, 480);

              // Get a CvSink. This will capture Mats from the camera
              CvSink cvSink = CameraServer.getVideo();
              // Setup a CvSource. This will send images back to the Dashboard
              CvSource outputStream = CameraServer.putVideo("Rectangle", 640, 480);

              // Mats are very memory expensive. Lets reuse this Mat.
              Mat mat = new Mat();

              // This cannot be 'true'. The program will never exit if it is. This
              // lets the robot stop this thread when restarting robot code or
              // deploying.
              while (!Thread.interrupted()) {
                // Tell the CvSink to grab a frame from the camera and put it
                // in the source mat.  If there is an error notify the output.
                if (cvSink.grabFrame(mat) == 0) {
                  // Send the output the error.
                  outputStream.notifyError(cvSink.getError());
                  // skip the rest of the current iteration
                  continue;
                }
                // Put a rectangle on the image
                Imgproc.rectangle(
                    mat, new Point(100, 100), new Point(400, 400), new Scalar(255, 255, 255), 5);
                // Give the output stream a new image to display
                outputStream.putFrame(mat);
              }
            });
		m_visionThread.setDaemon(true);
		m_visionThread.start();
		

		/*
		// -------------------
		// SINGLE CAMERA FEED SETUP WORKS
		
		new Thread(() -> {
			UsbCamera usbCamera0 = CameraServer.startAutomaticCapture();
			// Creates UsbCamera and MjpegServer [1] and connects them
			
			MjpegServer mjpegServer1 = new MjpegServer("serve_USB Camera 0", 1181);
			mjpegServer1.setSource(usbCamera0);

			// Creates the CvSink and connects it to the UsbCamera
			CvSink cvSink = new CvSink("opencv_USB Camera 0");
			cvSink.setSource(usbCamera0);

			// Creates the CvSource and MjpegServer [2] and connects them
			CvSource outputStream = new CvSource("Blur0", PixelFormat.kMJPEG, 640, 480, 30);
			MjpegServer mjpegServer2 = new MjpegServer("serve_Blur0", 1182);
			mjpegServer2.setSource(outputStream);

			Mat source = new Mat();
      		Mat output = new Mat();

			while(!Thread.interrupted()) {
				if (cvSink.grabFrame(source) == 0) {
				  continue;
				}
				Imgproc.cvtColor(source, output, Imgproc.COLOR_BGR2GRAY);
				outputStream.putFrame(output);
			}


		}).start();

		// ---- END CAMERA FEEDS
		// ----------------------
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
			DriveMode.GOALMODE,  () -> copilot.getXButton(),
			DriveMode.BALLMODE,  () -> copilot.getYButton(),
			DriveMode.ZERO_GYRO, () -> copilot.getStartButton()
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
