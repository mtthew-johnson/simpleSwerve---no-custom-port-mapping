package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.TalonSRXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotBase;

public class SwerveDrive extends SubsystemBase {

    private final double SPEED = 0.7;
    
    private double KpDrivefl = 0.04; //TODO find pid inputs for all pid controllers
	private double KiDrivefl = 0;
	private double KdDrivefl = 0;//0.002;

    private double KpDrivefr = 0;
	private double KiDrivefr = 0;
	private double KdDrivefr = 0;
    
    private double KpDrivebl = 0;
	private double KiDrivebl = 0;
	private double KdDrivebl = 0;

    private double KpDrivebr = 0;
	private double KiDrivebr = 0;
	private double KdDrivebr = 0;
    
    private double KpAnglefl = 0;
	private double KiAnglefl = 0;
	private double KdAnglefl = 0;

    private double KpAnglefr = 0;
	private double KiAnglefr = 0;
	private double KdAnglefr = 0;

    private double KpAnglebl = 0;
	private double KiAnglebl = 0;
	private double KdAnglebl = 0;

    private double KpAnglebr = 0;
	private double KiAnglebr = 0;
	private double KdAnglebr = 0;
	
    // private double correction = 0;
	// private double prevAngle = 0;
	// private double angleCompensation = 0;

    private WPI_TalonSRX frontRightSpeedMotor;
    private WPI_TalonSRX frontLeftSpeedMotor;
    private WPI_TalonSRX backRightSpeedMotor;
    private WPI_TalonSRX backLeftSpeedMotor;
    
    private WPI_TalonSRX frontRightAngleMotor;
    private WPI_TalonSRX frontLeftSAngleMotor;
    private WPI_TalonSRX backRightAngleMotor;
    private WPI_TalonSRX backLeftAngleMotor;

    // private boolean drivePOV = false;
    private boolean safeMode = false;

    private ADXRS450_Gyro gyro;

    private Encoder frontLeftEncoder;
    private Encoder frontRightEncoder;
    private Encoder backLeftEncoder;
    private Encoder backRightEncoder;
    
    private PIDController pidDrivefl;
    private PIDController pidDrivefr;
    private PIDController pidDrivebl;
    private PIDController pidDrivebr;
    
    private PIDController pidAnglefl;
    private PIDController pidAnglefr;
    private PIDController pidAnglebl;
    private PIDController pidAnglebr;

    public SwerveDrive(RobotBase robot) {
        
        super(robot);
       
        initMotors();
        configureGyro();
        configureEncoders();

        configurePID();

        reset();
        initDefaultCommand();
        
    }

    private void initMotors() {
        
        frontRightSpeedMotor = new WPI_TalonSRX(port("frontRightSpeedMotor"));
        frontLeftSpeedMotor  = new WPI_TalonSRX(port("frontLeftSpeedMotor"));
        backRightSpeedMotor  = new WPI_TalonSRX(port("backRightSpeedMotor"));
        backLeftSpeedMotor   = new WPI_TalonSRX(port("backLeftSpeedMotor"));
        
        frontRightAngleMotor = new WPI_TalonSRX(port("frontRightAngleMotor"));
        frontLeftSAngleMotor = new WPI_TalonSRX(port("frontLeftSAngleMotor"));
        backRightAngleMotor  = new WPI_TalonSRX(port("backRightAngleMotor"));
        backLeftAngleMotor   = new WPI_TalonSRX(port("backLeftAngleMotor"));

        addChild("frontRightSpeedMotor", frontRightSpeedMotor);
        addChild("frontLeftSpeedMotor",  frontLeftSpeedMotor);
        addChild("backRightSpeedMotor",  backRightSpeedMotor);
        addChild("backLeftSpeedMotor",   backLeftSpeedMotor);

        addChild("frontRightAngleMotor", frontRightAngleMotor);
        addChild("frontLeftSAngleMotor", frontLeftSAngleMotor);
        addChild("backRightAngleMotor",  backRightAngleMotor);
        addChild("backLeftAngleMotor",   backLeftAngleMotor);

        frontRightSpeedMotor.setInverted(false);
        frontLeftSpeedMotor.setInverted(false);
        backRightSpeedMotor.setInverted(false);
        backLeftSpeedMotor.setInverted(false);

        frontRightAngleMotor.setInverted(false);
        frontLeftSAngleMotor.setInverted(false);
        backRightAngleMotor.setInverted(false);
        backLeftAngleMotor.setInverted(true);
    
        frontRightAngleMotor.configSelectedFeedbackSensor(FeedbackDevice.Analog);
        frontLeftSAngleMotor.configSelectedFeedbackSensor(FeedbackDevice.Analog);
        backRightAngleMotor.configSelectedFeedbackSensor(FeedbackDevice.Analog);
        backLeftAngleMotor.configSelectedFeedbackSensor(FeedbackDevice.Analog);
        
        frontRightSpeedMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
        frontLeftSpeedMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
        backRightSpeedMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
        backLeftSpeedMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);


        // frontRightAngleMotor.set(TalonSRXControlMode.Position, 0);
        // frontLeftSAngleMotor.set(TalonSRXControlMode.Position, 0);
        // backRightAngleMotor.set(TalonSRXControlMode.Position,  0);
        // backLeftAngleMotor.set(TalonSRXControlMode.Position,   0);

        frontRightAngleMotor.setSelectedSensorPosition(0);
        frontLeftSAngleMotor.setSelectedSensorPosition(0);
        backRightAngleMotor.setSelectedSensorPosition(0);
        backLeftAngleMotor.setSelectedSensorPosition(0);
        
        // frontRightSpeedMotor.set(ControlMode.PercentOutput, 0);
        // frontLeftSpeedMotor.set(ControlMode.PercentOutput,  0);
        // backLeftSpeedMotor.set(ControlMode.PercentOutput,   0);
        // backRightSpeedMotor.set(ControlMode.PercentOutput,  0);
    
    }
    
    private void calculateDrive(double FWD, double STR, double RCW, double gryroAngle) {
       
        double temp = FWD*Math.cos(gryroAngle) + STR*Math.sin(gryroAngle);
        
        STR = -FWD*Math.sin(gryroAngle) + STR*Math.cos(gryroAngle);
        FWD = temp;

        final double PI = Math.PI;
        final double L = 23; //TODO find vehicle wheelbase //units don't matter as it's a ratio
        final double W = 23; //TODO find vehicle trackwidth
        final double R = Math.sqrt(Math.pow(L, 2) + Math.pow(W, 2));

        double A = STR - RCW*(L/R);
        double B = STR + RCW*(L/R);
        double C = FWD - RCW*(W/R);
        double D = FWD + RCW*(W/R); 

        double frontRightWheelSpeed = Math.sqrt(Math.pow(B, 2) + Math.pow(C, 2));
        double frontLeftWheelSpeed  = Math.sqrt(Math.pow(B, 2) + Math.pow(D, 2));
        double backLeftWheelSpeed   = Math.sqrt(Math.pow(A, 2) + Math.pow(D, 2));
        double backRightWheelSpeed  = Math.sqrt(Math.pow(A, 2) + Math.pow(C, 2));

        double frontRightWheelAngle = Math.atan2(B, C) * (180 / PI);
        double frontLeftWheelAngle  = Math.atan2(B, D) * (180 / PI);
        double backLeftWheelAngle   = Math.atan2(A, D) * (180 / PI);
        double backRightWheelAngle  = Math.atan2(A, C) * (180 / PI);

        double max = frontRightWheelSpeed;

        //normalize wheel speeds
        if(frontLeftWheelSpeed > max) {
            
            max = frontLeftWheelSpeed;
        
        } else if(backLeftWheelSpeed > max) {
            
            max = backLeftWheelSpeed;

        } else if (backRightWheelSpeed > max) {
            
            max = backRightWheelSpeed;

        } else {

        }

        // double speedCorrection1 = pidDrivefr.calculate(frontRightSpeedMotor.getMotorOutputPercent(), max * SPEED);
        // double speedCorrection2 = pidDrivefl.calculate(frontLeftSpeedMotor.getMotorOutputPercent(),  max * SPEED);
        // double speedCorrection3 = pidDrivebl.calculate(backLeftSpeedMotor.getMotorOutputPercent(),   max * SPEED);
        // double speedCorrection4 = pidDrivebr.calculate(backLeftSpeedMotor.getMotorOutputPercent(),   max * SPEED);

        frontRightWheelSpeed = (max * SPEED); //+ speedCorrection1;
        frontLeftWheelSpeed  = (max * SPEED); //+ speedCorrection2;
        backLeftWheelSpeed   = (max * SPEED); //+ speedCorrection3;
        backRightWheelSpeed  = (max * SPEED); //+ speedCorrection4;

        if(frontLeftWheelSpeed <= 0.05) {
            
            max = 0;
        
        } else if(backLeftWheelSpeed <= 0.05) {
            
            max = 0;

        } else if (backRightWheelSpeed <= 0.05) {
            
            max = 0;

        } else if (frontRightWheelSpeed <= 0.05) {
            
            max = 0;
        
        } else {

        }

        frontRightWheelSpeed = (max * SPEED); //+ speedCorrection1;
        frontLeftWheelSpeed  = (max * SPEED); //+ speedCorrection2;
        backLeftWheelSpeed   = (max * SPEED); //+ speedCorrection3;
        backRightWheelSpeed  = (max * SPEED); //+ speedCorrection4;
        
        if(axis("forward") > 0) {
            //set wheel speed
            frontRightSpeedMotor.set(frontRightWheelSpeed);
            frontLeftSpeedMotor.set(frontLeftWheelSpeed);
            backRightSpeedMotor.set(backLeftWheelSpeed);
            backLeftSpeedMotor.set(backRightWheelSpeed);
        } else if (axis("forward") < 0) {
            //set wheel speed
            frontRightSpeedMotor.set(-frontRightWheelSpeed);
            frontLeftSpeedMotor.set(-frontLeftWheelSpeed);
            backRightSpeedMotor.set(-backLeftWheelSpeed);
            backLeftSpeedMotor.set(-backRightWheelSpeed);
        } else {
            frontRightSpeedMotor.set(0);
            frontLeftSpeedMotor.set(0);
            backRightSpeedMotor.set(0);
            backLeftSpeedMotor.set(0);
        }
        
        

        //convert angle to talon ticks to set final angle/ 1024 ticks
        frontRightWheelAngle = ((frontRightWheelAngle + 180) * (1023 / 360));
        frontLeftWheelAngle  = ((frontLeftWheelAngle  + 180) * (1023 / 360));
        backLeftWheelAngle   = ((backLeftWheelAngle   + 180) * (1023 / 360));
        backRightWheelAngle  = ((backRightWheelAngle  + 180) * (1023 / 360));
        
        //set wheel angle and pid calculations
        // double correction1 = pidAnglefr.calculate(frontRightAngleMotor.getSelectedSensorPosition(), frontRightWheelAngle);
        // double correction2 = pidAnglefl.calculate(frontLeftSAngleMotor.getSelectedSensorPosition(), frontLeftWheelAngle);
        //double correction3 = pidAnglebl.calculate(backLeftEncoder.getDistance(),   180);
        // double correction4 = pidAnglebr.calculate(backRightAngleMotor.getSelectedSensorPosition(),  backRightWheelAngle);

        // frontRightAngleMotor.set (TalonSRXControlMode.Position, 300); //+ correction1);
        // frontLeftSAngleMotor.set(ControlMode.Position,);  //+ correction2);
        // backRightAngleMotor.set(ControlMode.Position, backLeftWheelAngle );  //+ correction3);
        // backLeftAngleMotor.set(ControlMode.Position,  backRightWheelAngle ); //+ correction4);
        
        // if(backLeftEncoder.getDistance() < 180) {
        //     backLeftAngleMotor.set(0.5);
        // } else if (backLeftEncoder.getDistance() > 180) {
        //     backLeftAngleMotor.set(-0.5);
        // } else {
        //     backLeftAngleMotor.set(0);
        // }

        // if(backRightEncoder.getDistance() < 180) {
        //     backRightAngleMotor.set(0.5);
        // } else if (backRightEncoder.getDistance() > 180) {
        //     backRightAngleMotor.set(-0.5);
        // } else {
        //     backRightAngleMotor.set(0);
        // }

        // if(frontLeftEncoder.getDistance() < 180) {
        //     frontLeftSAngleMotor.set(0.5);
        // } else if (frontLeftEncoder.getDistance() > 180) {
        //     frontLeftSAngleMotor.set(-0.5);
        // } else {
        //     frontLeftSAngleMotor.set(0);
        // }

        // if(frontRightEncoder.getDistance() < 180) {
        //     frontRightAngleMotor.set(0.5);
        // } else if (frontRightEncoder.getDistance() > 180) {
        //     frontRightAngleMotor.set(-0.5);
        // } else {
        //     frontRightAngleMotor.set(0);
        // }

        backLeftAngleMotor.set(pidAnglebl.calculate(backLeftEncoder.getDistance(), 180)); //TODO the .calculate() method for some reason outputs 0
        System.out.println(pidAnglebl.calculate(backLeftEncoder.getDistance(), 180));      //so the motors won't move
                                                                                            //jonathan, you will be my savior if you can get this to work
                                                                                            // check the documentation for PID controller class for more info and example implementation
                                                                                            //https://docs.wpilib.org/en/stable/docs/software/advanced-controls/controllers/pidcontroller.html
                                                                                            //also documention for encoder class if any encoder stuff doesn't make any sense:
                                                                                            //https://first.wpi.edu/FRC/roborio/release/docs/java/edu/wpi/first/wpilibj/Encoder.html


        System.out.println("encoder:  " + backLeftEncoder.getDistance());
        // System.out.println("frontRightWheelAngle: " + frontRightWheelAngle);
        // System.out.println("frontLeftWheelAngle: " + frontLeftWheelAngle);
        // System.out.println("backLeftWheelAngle: " + backLeftWheelAngle);
        // System.out.println("backRightWheelAngle: " + backRightWheelAngle);

        //System.out.println(frontLeftSAngleMotor.getControlMode());


        // System.out.println("frontRightWheelSpeed: " + frontRightWheelSpeed);
        // System.out.println("frontLeftWheelSpeed:  " + frontLeftWheelSpeed);
        // System.out.println("backLeftWheelSpeed:   " + backLeftWheelSpeed);
        // System.out.println("backRightWheelSpeed:  " + backRightWheelSpeed);

        //System.out.println(axis("forward"));
        //System.out.println(gyro.getAngle());


        
    }

    private void configureGyro() {
		
		gyro = new ADXRS450_Gyro();
		gyro.reset();

	}

    private void configurePID() {
		
		pidDrivefl = new PIDController(KpDrivefl, KiDrivefl, KdDrivefl);
		pidDrivefr = new PIDController(KpDrivefr, KiDrivefr, KdDrivefr);
		pidDrivebl = new PIDController(KpDrivebl, KiDrivebl, KdDrivebl);
		pidDrivebr = new PIDController(KpDrivebr, KiDrivebr, KdDrivebr);

        pidAnglefl = new PIDController(KpAnglefl, KiAnglefl, KdAnglefl);
        pidAnglefr = new PIDController(KpAnglefr, KiAnglefr, KdAnglefr);
          pidAnglebl = new PIDController(KpAnglebl, KiAnglebl, KdAnglebl);
        pidAnglebr = new PIDController(KpAnglebr, KiAnglebr, KdAnglebr);

	}

    private void configureEncoders() {
       frontRightEncoder = new Encoder(4, 5);
       frontLeftEncoder  = new Encoder(6, 7);
       backRightEncoder  = new Encoder(2, 3);
       backLeftEncoder   = new Encoder(0, 1);
        final double TICKSTOGEGREES = 1.12;
        //each 0.711 ticks is about one degree of rotation (406/360)
        frontRightEncoder.setDistancePerPulse(1./TICKSTOGEGREES);
        frontLeftEncoder.setDistancePerPulse(1./TICKSTOGEGREES);
        backRightEncoder.setDistancePerPulse(1./TICKSTOGEGREES);
        backLeftEncoder.setDistancePerPulse(1./TICKSTOGEGREES);
    }

	@Override
	public void periodic() {
		
	}

	public void stop() {

	}

	public void reset() {
		
	}

    protected void initDefaultCommand() {
        setDefaultCommand(new CommandBase() {

			double comboStartTime = 0;
			boolean alreadyToggled = false;

			{
				addRequirements(SwerveDrive.this);
				SendableRegistry.addChild(SwerveDrive.this, this);
			}

			@Override
			public void execute() {
                if (button("safeModeToggle")) {

					if (comboStartTime == 0)
						comboStartTime = Timer.getFPGATimestamp();
					else if (Timer.getFPGATimestamp() - comboStartTime >= 3.0 && !alreadyToggled) {
					
						safeMode = !safeMode;
						alreadyToggled = true;
						System.out.println("Safemode is " + (safeMode ? "Enabled" : "Disabled") + ".");
					
					}

				} else {

					comboStartTime = 0;
					alreadyToggled = false;
				
				}


                calculateDrive(axis("forward"), axis("strafe"), axis("rotate"), gyro.getAngle());
				
			}

		}); // End set default command

	} // End init default command

    @Override
	public String getConfigName() {
		return "swerve";
	}

	@Override
	public void initSendable(SendableBuilder builder) {

		super.initSendable(builder);

		builder.addDoubleProperty("PDrivefl", () -> KpDrivefl, (value) -> KpDrivefl = value);
		builder.addDoubleProperty("IDrivefl", () -> KiDrivefl, (value) -> KiDrivefl = value);
		builder.addDoubleProperty("DDrivefl", () -> KdDrivefl, (value) -> KdDrivefl = value);

        builder.addDoubleProperty("PDrivefr", () -> KpDrivefr, (value) -> KpDrivefr = value);
		builder.addDoubleProperty("IDrivefr", () -> KiDrivefr, (value) -> KiDrivefr = value);
		builder.addDoubleProperty("DDrivefr", () -> KdDrivefr, (value) -> KdDrivefr = value);

        builder.addDoubleProperty("PDrivefl", () -> KpDrivebl, (value) -> KpDrivebl = value);
		builder.addDoubleProperty("IDrivefl", () -> KiDrivebl, (value) -> KiDrivebl = value);
		builder.addDoubleProperty("DDrivefl", () -> KdDrivebl, (value) -> KdDrivebl = value);

        builder.addDoubleProperty("PDrivefl", () -> KpDrivebr, (value) -> KpDrivebr = value);
		builder.addDoubleProperty("IDrivefl", () -> KiDrivebr, (value) -> KiDrivebr = value);
		builder.addDoubleProperty("DDrivefl", () -> KdDrivebr, (value) -> KdDrivebr = value);

        builder.addDoubleProperty("PAngle", () -> KpAnglefl, (value) -> KpAnglefl = value);
		builder.addDoubleProperty("IAngle", () -> KiAnglefl, (value) -> KiAnglefl = value);
		builder.addDoubleProperty("DAngle", () -> KdAnglefl, (value) -> KdAnglefl = value);

        builder.addDoubleProperty("PAngle", () -> KpAnglefr, (value) -> KpAnglefr = value);
		builder.addDoubleProperty("IAngle", () -> KiAnglefr, (value) -> KiAnglefr = value);
		builder.addDoubleProperty("DAngle", () -> KdAnglefr, (value) -> KdAnglefr = value);

        builder.addDoubleProperty("PAngle", () -> KpAnglebl, (value) -> KpAnglebl = value);
		builder.addDoubleProperty("IAngle", () -> KiAnglebl, (value) -> KiAnglebl = value);
		builder.addDoubleProperty("DAngle", () -> KdAnglebl, (value) -> KdAnglebl = value);

        builder.addDoubleProperty("PAngle", () -> KpAnglebr, (value) -> KpAnglebr = value);
		builder.addDoubleProperty("IAngle", () -> KiAnglebr, (value) -> KiAnglebr = value);
		builder.addDoubleProperty("DAngle", () -> KdAnglebr, (value) -> KdAnglebr = value);

		builder.addDoubleProperty("Forward", () -> axis("forward"), null);
		builder.addDoubleProperty("Strafe", () -> axis("strafe"), null);
		builder.addDoubleProperty("Rotate", () -> axis("Rotate"), null);
		
        // builder.addDoubleProperty("prevAngle", () -> prevAngle, null);
		// builder.addBooleanProperty("isPOV", () -> drivePOV, null);
		// builder.addDoubleProperty("angleCompensation", () -> angleCompensation, null);
		builder.addBooleanProperty("isSafeMode", () -> safeMode, null);
		
        builder.addDoubleProperty("frontRightSpeedMotor", () -> frontRightSpeedMotor.get(), null);		
		builder.addDoubleProperty("frontLeftSpeedMotor",  () -> frontLeftSpeedMotor.get(),  null);
		builder.addDoubleProperty("backRightSpeedMotor",  () -> backRightSpeedMotor.get(),  null);
		builder.addDoubleProperty("backLeftSpeedMotor",   () -> backLeftSpeedMotor.get(),   null);
        
        builder.addDoubleProperty("frontRightAngleMotor", () -> frontRightAngleMotor.get(), null);		
		builder.addDoubleProperty("frontLeftSAngleMotor", () -> frontLeftSAngleMotor.get(), null);
		builder.addDoubleProperty("backRightAngleMotor",  () -> backRightAngleMotor.get(),  null);
		builder.addDoubleProperty("backLeftAngleMotor",   () -> backLeftAngleMotor.get(),   null);
		
	}
    
}
