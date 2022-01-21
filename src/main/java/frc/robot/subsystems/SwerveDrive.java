package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotBase;

public class SwerveDrive extends SubsystemBase {

    private final double SPEED = 0.7;

    private double KpAnglefl = 0.04;
	private double KiAnglefl = 0;
	private double KdAnglefl = 0;

    private double KpAnglefr = 0.04;
	private double KiAnglefr = 0;
	private double KdAnglefr = 0;

    private double KpAnglebl = 0.04;
	private double KiAnglebl = 0;
	private double KdAnglebl = 0;

    private double KpAnglebr = 0.04;
	private double KiAnglebr = 0;
	private double KdAnglebr = 0;

    private WPI_TalonSRX frontRightSpeedMotor;
    private WPI_TalonSRX frontLeftSpeedMotor;
    private WPI_TalonSRX backRightSpeedMotor;
    private WPI_TalonSRX backLeftSpeedMotor;
    
    private WPI_TalonSRX frontRightAngleMotor;
    private WPI_TalonSRX frontLeftSAngleMotor;
    private WPI_TalonSRX backRightAngleMotor;
    private WPI_TalonSRX backLeftAngleMotor;

    private boolean safeMode = false;

    private ADXRS450_Gyro gyro;

    private Encoder frontLeftEncoder;
    private Encoder frontRightEncoder;
    private Encoder backLeftEncoder;
    private Encoder backRightEncoder;
    
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

        frontRightSpeedMotor.setInverted(true);
        frontLeftSpeedMotor.setInverted(true);
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

        backLeftAngleMotor.configOpenloopRamp(0);
        backLeftAngleMotor.configClosedloopRamp(0);

        backRightAngleMotor.configOpenloopRamp(0);
        backRightAngleMotor.configClosedloopRamp(0);

        frontLeftSAngleMotor.configOpenloopRamp(0);
        frontLeftSAngleMotor.configClosedloopRamp(0);

        frontRightAngleMotor.configOpenloopRamp(0);
        frontRightAngleMotor.configClosedloopRamp(0);
    }

    private void optmizeAngle(Encoder angleEncoder, double targetAngle, double currentAngle, String encoderName) { 
        
    }
    
    private void calculateDrive(double FWD, double STR, double RCW, double gryroAngle) {
       
        double temp = FWD*Math.cos(gryroAngle) + STR*Math.sin(gryroAngle);
        
        STR = -FWD*Math.sin(gryroAngle) + STR*Math.cos(gryroAngle);
        FWD = temp;

        final double PI = Math.PI;
        final double L = 23;
        final double W = 23; 
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
        
        backLeftAngleMotor.set(pidAnglebl.calculate(backLeftEncoder.getDistance(), backLeftWheelAngle));
        backRightAngleMotor.set(pidAnglebr.calculate(backRightEncoder.getDistance(), backRightWheelAngle));
        frontRightAngleMotor.set(pidAnglefr.calculate(frontRightEncoder.getDistance(), frontRightWheelAngle));
        frontLeftSAngleMotor.set(pidAnglefl.calculate(frontLeftEncoder.getDistance(), frontLeftWheelAngle));
        
    }

    private void configureGyro() {
		
		gyro = new ADXRS450_Gyro();
		gyro.reset();

	}

    private void configurePID() {

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
        
        final double TICKS_TO_DEGREES = 1.12;

        frontRightEncoder.setDistancePerPulse(1./TICKS_TO_DEGREES);
        frontLeftEncoder.setDistancePerPulse(1./TICKS_TO_DEGREES);
        backRightEncoder.setDistancePerPulse(1./TICKS_TO_DEGREES);
        backLeftEncoder.setDistancePerPulse(1./TICKS_TO_DEGREES);
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

                double forwardtemp = axis("forward");
                double strafetemp = axis("strafe");
                double rotatetemp = axis("rotate");
                
                //set controller deadzones
                if(Math.abs(axis("forward")) < 0.1) {
                   
                    forwardtemp = 0;

                } else {
                    
                    forwardtemp = axis("forward");
                }

                if (Math.abs(axis("strafe")) < 0.2) {
                    
                    strafetemp = 0;

                } else {
                    
                    strafetemp = axis("strafe");

                }
               
                if (Math.abs(axis("rotate")) < 0.2) {
                    
                    rotatetemp = 0;

                } else {
                    
                    rotatetemp = axis("rotate");

                }

                System.out.println("forwardtemp: " + forwardtemp);
                System.out.println("strafetemp:  " + strafetemp);
                System.out.println("rotatetemp:  " + rotatetemp);

                calculateDrive(forwardtemp, strafetemp, rotatetemp, gyro.getAngle());
				
			}

		}); // End set default command

	} // End init default command

	@Override
	public void initSendable(SendableBuilder builder) {

		super.initSendable(builder);

        builder.addDoubleProperty("PAnglefl", () -> KpAnglefl, (value) -> KpAnglefl = value);
		builder.addDoubleProperty("IAnglefl", () -> KiAnglefl, (value) -> KiAnglefl = value);
		builder.addDoubleProperty("DAnglefl", () -> KdAnglefl, (value) -> KdAnglefl = value);

        builder.addDoubleProperty("PAnglefr", () -> KpAnglefr, (value) -> KpAnglefr = value);
		builder.addDoubleProperty("IAnglefr", () -> KiAnglefr, (value) -> KiAnglefr = value);
		builder.addDoubleProperty("DAnglefr", () -> KdAnglefr, (value) -> KdAnglefr = value);

        builder.addDoubleProperty("PAnglebl", () -> KpAnglebl, (value) -> KpAnglebl = value);
		builder.addDoubleProperty("IAnglebl", () -> KiAnglebl, (value) -> KiAnglebl = value);
		builder.addDoubleProperty("DAnglebl", () -> KdAnglebl, (value) -> KdAnglebl = value);

        builder.addDoubleProperty("PAnglebr", () -> KpAnglebr, (value) -> KpAnglebr = value);
		builder.addDoubleProperty("IAnglebr", () -> KiAnglebr, (value) -> KiAnglebr = value);
		builder.addDoubleProperty("DAnglebr", () -> KdAnglebr, (value) -> KdAnglebr = value);

		builder.addDoubleProperty("Forward", () -> axis("forward"), null);
		builder.addDoubleProperty("Strafe",  () -> axis("strafe"),  null);
		builder.addDoubleProperty("Rotate",  () -> axis("Rotate"),  null);
		
        builder.addDoubleProperty("frontRightEncoder", () -> frontRightEncoder.getDistance(), null);		
		builder.addDoubleProperty("frontLeftEncoder",  () -> frontLeftEncoder.getDistance(),  null);
		builder.addDoubleProperty("backRightEncoder",  () -> backRightEncoder.getDistance(),  null);
		builder.addDoubleProperty("backLeftEncoder",   () -> backLeftEncoder.getDistance(),   null);

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
