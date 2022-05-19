package frc.team5973.robot.subsystems;

//pheonix libraries imports
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.sensors.Pigeon2;

//WPILib imports
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team5973.robot.Gains;
public class SwerveDrive extends SubsystemBase {
    
    public Pigeon2 gyro; //declare gyro

    private final int timeoutMs = 30;

    private double KpAnglefl = 0.015; //PID values for each rotation encoder
	private double KiAnglefl = 0; //TODO: adjust as necessary
	private double KdAnglefl = 0;

    private double KpAnglefr = 0.015; //PID values for each rotation encoder
	private double KiAnglefr = 0; //TODO: adjust as necessary
	private double KdAnglefr = 0;

    private double KpAnglebl = 0.015; //PID values for each rotation encoder
	private double KiAnglebl = 0; //TODO: adjust as necessary
	private double KdAnglebl = 0;

    private double KpAnglebr = 0.015; //PID values for each rotation encoder
	private double KiAnglebr = 0; //TODO: adjust as necessary
	private double KdAnglebr = 0;
                                                            //kP,  kI,   kD  kF               kIzone kPeakOutput
    public final static Gains kGains_Velocitfl  = new Gains( 0.1, 0.001, 5,  1023.0/20660.0,  300,   1.00); //PID values for the falcon encoders 
    public final static Gains kGains_Velocitfr  = new Gains( 0.1, 0.001, 5,  1023.0/20660.0,  300,   1.00); //go look at the pheonix documentation for examples of this but these values worked pretty well on our falcons
    public final static Gains kGains_Velocitbr  = new Gains( 0.1, 0.001, 5,  1023.0/20660.0,  300,   1.00);
    public final static Gains kGains_Velocitbl  = new Gains( 0.1, 0.001, 5,  1023.0/20660.0,  300,   1.00);

    private WPI_TalonFX frontRightSpeedMotor; //declare drive motors
    private WPI_TalonFX frontLeftSpeedMotor;
    private WPI_TalonFX backRightSpeedMotor;
    private WPI_TalonFX backLeftSpeedMotor;
    
    private WPI_TalonSRX frontRightAngleMotor; //declare rotation motors
    private WPI_TalonSRX frontLeftAngleMotor;
    private WPI_TalonSRX backRightAngleMotor;
    private WPI_TalonSRX backLeftAngleMotor;

    // private Encoder frontLeftEncoder; //use these encoder delcarations if you have encoders directly plugged into dio ports and not your motor controllers
    // private Encoder frontRightEncoder;
    // private Encoder backLeftEncoder;
    // private Encoder backRightEncoder;

    private boolean sensorPhasefl = false; //TODO //decide whether to invert the sensor values on the rotation encoder or not //you will also need to test this
    private boolean sensorPhasefr = true;
    private boolean sensorPhaseBr = false;
    private boolean sensorPhaseBl = false;

    private PIDController pidAnglefl; //declare PID controllers for the rotation encoders
    private PIDController pidAnglefr;
    private PIDController pidAnglebl;
    private PIDController pidAnglebr;

    private final double L  = 23; //TODO //vehicle tracklength //adjust for your robot
    private final double W  = 23; //TODO //vehicle trackwidth
    private final double R  = Math.sqrt(Math.pow(L, 2) + Math.pow(W, 2)); //math needed for the swerve drive calculations
    private final double PI = Math.PI;

    public enum Axis {FORWARD, STRAFE, ROTATE} //controller inputs for swerve
    public enum DriveMode {SAFEMMODE, FIELDMODE, ZERO_GYRO} //button inputs 
    private double STRNew = 0; //variables for swerve drive calculations
    private double FWDNew = 0;

    private double storedHeading = 0; //variables for heading correction to allow the robot to drive
    private double correction = 0;


    public SwerveDrive() {
        
        configureMotors();
        configureGyro();
        configurePID();

        reset();        
    }

    private void configureMotors() {

        //This is how you would set up motors if you have a setup described in my paper; you may need to chagne this
        //TODO //declare ports for the motors //you need input yours
        frontRightSpeedMotor = new WPI_TalonFX(0);
        frontLeftSpeedMotor  = new WPI_TalonFX(0);
        backRightSpeedMotor  = new WPI_TalonFX(0);
        backLeftSpeedMotor   = new WPI_TalonFX(0);

        frontRightAngleMotor = new WPI_TalonSRX(0);
        frontLeftAngleMotor  = new WPI_TalonSRX(0);
        backRightAngleMotor  = new WPI_TalonSRX(0);
        backLeftAngleMotor   = new WPI_TalonSRX(0);

        //configure motors to their factory defaults
        frontRightAngleMotor.configFactoryDefault();
        frontLeftAngleMotor.configFactoryDefault();
        backRightAngleMotor.configFactoryDefault();
        backLeftAngleMotor.configFactoryDefault();

        frontRightSpeedMotor.configFactoryDefault();
        frontLeftSpeedMotor.configFactoryDefault();
        backRightSpeedMotor.configFactoryDefault();
        backLeftSpeedMotor.configFactoryDefault();
       
        //invert speed motors
        frontRightSpeedMotor.setInverted(true); //TODO: //this is also something you have to test and tun or your wheels will fight each other
        frontLeftSpeedMotor.setInverted(false);
        backRightSpeedMotor.setInverted(true);
        backLeftSpeedMotor.setInverted(false);

        //invert angle motors
        frontRightAngleMotor.setInverted(false); //TODO: //this is also something you have to test and tun or your wheels will fight each other
        frontLeftAngleMotor.setInverted(true);
        backRightAngleMotor.setInverted(true);
        backLeftAngleMotor.setInverted(true);
    
        //configure the rotation encoders correctly so that they can be read by the talons
        frontRightAngleMotor.configSelectedFeedbackSensor(FeedbackDevice.Analog);
        frontLeftAngleMotor.configSelectedFeedbackSensor(FeedbackDevice.Analog);
        backRightAngleMotor.configSelectedFeedbackSensor(FeedbackDevice.Analog);
        backLeftAngleMotor.configSelectedFeedbackSensor(FeedbackDevice.Analog);

        //set the sensor phase with the variables declared above
        frontRightAngleMotor.setSensorPhase(sensorPhasefr);
        frontLeftAngleMotor.setSensorPhase(sensorPhasefl);
        backLeftAngleMotor.setSensorPhase(sensorPhaseBl);
        backRightAngleMotor.setSensorPhase(sensorPhaseBr);
        
        //correctly configure the falcons so that their built in encoder works properly
        frontRightSpeedMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, timeoutMs);
        frontLeftSpeedMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor,  0, timeoutMs);
        backRightSpeedMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor,  0, timeoutMs);
        backLeftSpeedMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor,   0, timeoutMs);

        //config pid for falcons
        frontRightSpeedMotor.configNeutralDeadband(0.02);
        frontLeftSpeedMotor.configNeutralDeadband(0.02);
        backRightSpeedMotor.configNeutralDeadband(0.02);
        backLeftSpeedMotor.configNeutralDeadband(0.02);

        //config pid for falcons
        frontRightSpeedMotor.config_kF(0, kGains_Velocitfr.kF, timeoutMs);
		frontRightSpeedMotor.config_kP(0, kGains_Velocitfr.kP, timeoutMs);
		frontRightSpeedMotor.config_kI(0, kGains_Velocitfr.kI, timeoutMs);
		frontRightSpeedMotor.config_kD(0, kGains_Velocitfr.kD, timeoutMs); 
        
        //config pid for falcons
        frontLeftSpeedMotor.config_kF(0, kGains_Velocitfl.kF, timeoutMs);
		frontLeftSpeedMotor.config_kP(0, kGains_Velocitfl.kP, timeoutMs);
		frontLeftSpeedMotor.config_kI(0, kGains_Velocitfl.kI, timeoutMs);
		frontLeftSpeedMotor.config_kD(0, kGains_Velocitfl.kD, timeoutMs);
        
        //config pid for falcons
        backRightSpeedMotor.config_kF(0, kGains_Velocitbr.kF, timeoutMs);
		backRightSpeedMotor.config_kP(0, kGains_Velocitbr.kP, timeoutMs);
		backRightSpeedMotor.config_kI(0, kGains_Velocitbr.kI, timeoutMs);
		backRightSpeedMotor.config_kD(0, kGains_Velocitbr.kD, timeoutMs);  
        
        //config pid for falcons
        backLeftSpeedMotor.config_kF(0, kGains_Velocitbl.kF, timeoutMs);
		backLeftSpeedMotor.config_kP(0, kGains_Velocitbl.kP, timeoutMs);
		backLeftSpeedMotor.config_kI(0, kGains_Velocitbl.kI, timeoutMs);
		backLeftSpeedMotor.config_kD(0, kGains_Velocitbl.kD, timeoutMs);

        //turn off ramping for angleMotors
        backLeftAngleMotor.configOpenloopRamp(0);
        backLeftAngleMotor.configClosedloopRamp(0);

        backRightAngleMotor.configOpenloopRamp(0);
        backRightAngleMotor.configClosedloopRamp(0);

        frontLeftAngleMotor.configOpenloopRamp(0);
        frontLeftAngleMotor.configClosedloopRamp(0);

        frontRightAngleMotor.configOpenloopRamp(0);
        frontRightAngleMotor.configClosedloopRamp(0);

        //brake mode for speed motors
        backLeftSpeedMotor.setNeutralMode(NeutralMode.Brake);
        backRightSpeedMotor.setNeutralMode(NeutralMode.Brake);
        frontLeftSpeedMotor.setNeutralMode(NeutralMode.Brake);
        frontRightSpeedMotor.setNeutralMode(NeutralMode.Brake);

        //brake mode for angle motors
        backLeftAngleMotor.setNeutralMode(NeutralMode.Brake);
        backRightAngleMotor.setNeutralMode(NeutralMode.Brake);
        frontLeftAngleMotor.setNeutralMode(NeutralMode.Brake);
        frontRightAngleMotor.setNeutralMode(NeutralMode.Brake);

    }

    private void configureGyro() { //configures the gyro sensor
        gyro = new Pigeon2(0);

       gyro.setYaw(0, 69);

    }

    private void configurePID() {

        pidAnglefl = new PIDController(KpAnglefl, KiAnglefl, KdAnglefl); //sets up pid controllers for the rotation encoders
        pidAnglefr = new PIDController(KpAnglefr, KiAnglefr, KdAnglefr);
        pidAnglebl = new PIDController(KpAnglebl, KiAnglebl, KdAnglebl);
        pidAnglebr = new PIDController(KpAnglebr, KiAnglebr, KdAnglebr);  

	}

    // private void configureEncoders() { //use this method if using encoders that plug into DIO inputs

    //     frontRightEncoder = new Encoder(7, 6);//new Encoder(4, 5);
    //     frontLeftEncoder  = new Encoder(1, 0);//new Encoder(7, 6);
    //     backRightEncoder  =  new Encoder(5,4);//new Encoder(3, 2);
    //     backLeftEncoder   = new Encoder(3,2); //new Encoder(0, 1);
        
    //     final double TICKS_TO_DEGREES = 1.12;

    //     frontRightEncoder.setDistancePerPulse(1./TICKS_TO_DEGREES);
    //     frontLeftEncoder.setDistancePerPulse(1./TICKS_TO_DEGREES);
    //     backRightEncoder.setDistancePerPulse(1./TICKS_TO_DEGREES);
    //     backLeftEncoder.setDistancePerPulse(1./TICKS_TO_DEGREES);

    // }
    private void calculateDrive(double FWD, double STR, double RCW, boolean useGyro) {
               
        if(useGyro) {  //determine whether to use field centric orientation or not
            //gyro was inverted because of the position of our gyro on our robot; you might need to change this base on your gyro orientation
            double gyroAngle = -gyro.getYaw() * PI / 180; //you MUST convert the gyro angle to radians or field centric transformation won't work

            FWDNew = FWD*Math.cos(gyroAngle) + STR*Math.sin(gyroAngle); //uses the gyro to calcualte field centric transformation
            STRNew = STR*Math.cos(gyroAngle) - FWD*Math.sin(gyroAngle);

        } else {
            
            FWDNew = FWD*Math.cos(0) + STR*Math.sin(0); //passes zero instead of the gyro heading for robot oriented drive
            STRNew = STR*Math.cos(0) - FWD*Math.sin(0);

        }

        double A = STRNew - RCW*(L/R); //values a b c and d are used to calculate meaningful wheel speeds based on our inputs
        double B = STRNew + RCW*(L/R);
        double C = FWDNew - RCW*(W/R);
        double D = FWDNew + RCW*(W/R); 

        double frontRightWheelSpeed = Math.sqrt(Math.pow(B, 2) + Math.pow(C, 2)); //average the values of a b c and d to get wheel speeds
        double frontLeftWheelSpeed  = Math.sqrt(Math.pow(B, 2) + Math.pow(D, 2));
        double backLeftWheelSpeed   = Math.sqrt(Math.pow(A, 2) + Math.pow(D, 2));
        double backRightWheelSpeed  = Math.sqrt(Math.pow(A, 2) + Math.pow(C, 2));

        double frontRightWheelAngle = (Math.atan2(B, C) * (180 / PI)); // calculate the proper angles that each of the wheels need to be
        double frontLeftWheelAngle  = (Math.atan2(B, D) * (180 / PI));
        double backLeftWheelAngle   = (Math.atan2(A, D) * (180 / PI));
        double backRightWheelAngle  = (Math.atan2(A, C) * (180 / PI));

        //now we have our wheel speeds and angles, what do we do with them?

        //normalize wheel speeds
        double max = frontRightWheelSpeed;

        //find which wheel speed is the greatest
        if(frontLeftWheelSpeed > max) {
            max = frontLeftWheelSpeed;
        } else if(backLeftWheelSpeed > max) {
            max = backLeftWheelSpeed;
        } else if (backRightWheelSpeed > max) {
            max = backRightWheelSpeed;
        } else {}

        //set each wheel to the greatest wheel speed
        frontRightWheelSpeed = max; 
        frontLeftWheelSpeed  = max; 
        backLeftWheelSpeed   = max; 
        backRightWheelSpeed  = max;

        //set wheel angles and speeds
        //the way it's done here is to make sure the wheels have to turn the least amount of distance possible
        //take a look at the closestAngle method and the setDirection method that calcualte the most efficient position for the wheels to be in
        //your robot isn't going to drive very well
        //the best way to show why to do it this way is just to try and directly input the wheel speeds instead of these fun calcualtions

        // //uncomment this code and comment out lines 305-308
        
        // //set wheel angles
        // backLeftAngleMotor.set(pidAnglebl.calculate(ticksToDegrees(backLeftAngleMotor), backLeftWheelAngle));
        // backRightAngleMotor.set(pidAnglebl.calculate(ticksToDegrees(backLeftAngleMotor), backLeftWheelAngle));
        // frontRightAngleMotor.set(pidAnglebl.calculate(ticksToDegrees(backLeftAngleMotor), backLeftWheelAngle));
        // frontLeftAngleMotor.set(pidAnglebl.calculate(ticksToDegrees(backLeftAngleMotor), backLeftWheelAngle));

        // //sets wheel speeds
        // backLeftSpeedMotor.set(backLeftWheelSpeed);
        // backRightSpeedMotor.set(backRightWheelSpeed);
        // frontLeftSpeedMotor.set(frontLeftWheelSpeed);
        // frontRightSpeedMotor.set(frontRightWheelSpeed);

        //see much more elegant and less code
        backLeftAngleMotor.set(pidAnglebl.calculate(ticksToDegrees(backLeftAngleMotor),      setDirection(backLeftWheelAngle,   backLeftAngleMotor,   backLeftSpeedMotor,   backLeftWheelSpeed)));
        backRightAngleMotor.set(pidAnglebr.calculate(ticksToDegrees(backRightAngleMotor),    setDirection(backRightWheelAngle,  backRightAngleMotor,  backRightSpeedMotor,  backRightWheelSpeed)));
        frontRightAngleMotor.set(pidAnglefr.calculate(ticksToDegrees(frontRightAngleMotor),  setDirection(frontRightWheelAngle, frontRightAngleMotor, frontRightSpeedMotor, frontRightWheelSpeed)));
        frontLeftAngleMotor.set(pidAnglefl.calculate(ticksToDegrees(frontLeftAngleMotor),    setDirection(frontLeftWheelAngle,  frontLeftAngleMotor,  frontLeftSpeedMotor,  frontLeftWheelSpeed)));

    }

    //Get the closest angle between the given angles.
    private static double closestAngle(double a, double b) { //the value a being the current angle of the robot and the value b being the target angle of the robot
            // get direction
            double dir = (b % 360) - (a % 360);

            // convert from -360 to 360 to -180 to 180
            if(Math.abs(dir) > 180.0) {
                    dir = -(Math.signum(dir) * 360.0) + dir;
            }
            return dir;
    }

    //Sets the final position of the rotation motors
    private double setDirection(double setpoint, WPI_TalonSRX motor, WPI_TalonFX speedMotor, double speed) {
        
        double direction = 0;
        double currentAngle = ticksToDegrees(motor);
        // find closest angle to setpoint
        double setpointAngle = closestAngle(currentAngle, setpoint);
        // find closest angle to setpoint + 180
        double setpointAngleFlipped = closestAngle(currentAngle, setpoint + 180.0);
        // if the closest angle to setpoint is shorter
        if(Math.abs(setpointAngle) <= Math.abs(setpointAngleFlipped)) {
            // unflip the motor direction use the setpoint
            direction = currentAngle + setpointAngle;
        }
        // if the closest angle to setpoint + 180 is shorter
        else {
            // flip the motor direction and use the setpoint + 180
            speed = -speed;
            direction = currentAngle + setpointAngleFlipped;
        }

        speedMotor.set(TalonFXControlMode.PercentOutput, speed); //set the motor speed
        return direction;
    }
    
    //converts ticks to degrees so the encoder values are easy to read
    private double ticksToDegrees(WPI_TalonSRX motor) {
        return motor.getSelectedSensorPosition() / (2.844444444444444444444444444444444444444444444);
    }
    
    //basically a control loop to determine the amount of correction needed to correct the robot to keep it straight
    private double calcYawStraight(double targetAngle, double currentAngle, double kP) {
        double errorAngle = (targetAngle - (currentAngle % 360));
        double correction = errorAngle * kP;

        return correction;
    }

     //method that does the heading correction depending on whether the robot is currently rotating or not
     public double correctHeading(double kP, double FWD, double STR, double RCW) {
        if(RCW != 0) { //if the robot is being rotated by the driver, store a new heading to correct to
            storedHeading = -gyro.getYaw();
        } else {
            //System.out.println(STR);
            if(Math.abs(FWD) > 0 || Math.abs(STR) > 0) {
                correction = calcYawStraight(storedHeading, -gyro.getYaw(), kP);

                if(Math.abs(correction) > 1) {
                    correction = 0;
                }
            }
        }

        return correction;
    }

    public double getGyroAngle() { //gets the current gyyro angle
        return -gyro.getYaw();
    }

    public void resetGyro() { //sets current gyro heading to 0
        gyro.setYaw(0, 100);
    }

    //the method that actually gets used for the swerve default command
    public void swerveDrive(double FWD, double STR, double RCW, boolean useGyro) {
        calculateDrive(FWD, STR, RCW, useGyro);
    }

	@Override
	public void periodic() {
		
	}

	public void stop() {    

	}

	public void reset() {
		
	}
	@Override
	public void initSendable(SendableBuilder builder) { //data sent to the smart dashboard for debugging purposes //add or delete as necessary

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

		builder.addDoubleProperty("Forward", () ->   FWDNew, null);
		builder.addDoubleProperty("Strafe",  () ->   STRNew, null);
		// builder.addDoubleProperty("Rotate",  () ->   MathUtil.applyDeadband(axis("rotate"),  DEADBAND), null);

        builder.addDoubleProperty("Gyro Angle", () -> -gyro.getYaw(), null);
        
        builder.addDoubleProperty("Front Right Angle", () -> ticksToDegrees(frontRightAngleMotor), null);		
		builder.addDoubleProperty("Front Left Angle",  () -> ticksToDegrees(frontLeftAngleMotor),  null);
		builder.addDoubleProperty("Back Right Angle",  () -> ticksToDegrees(backRightAngleMotor),  null);
		builder.addDoubleProperty("Back Left Angle",   () -> ticksToDegrees(backLeftAngleMotor),   null);
	
        builder.addDoubleProperty("frontRightSpeedMotor", () -> frontRightSpeedMotor.get(), null);		
		builder.addDoubleProperty("frontLeftSpeedMotor",  () -> frontLeftSpeedMotor.get(),  null);
		builder.addDoubleProperty("backRightSpeedMotor",  () -> backRightSpeedMotor.get(),  null);
		builder.addDoubleProperty("backLeftSpeedMotor",   () -> backLeftSpeedMotor.get(),   null);
        
        builder.addDoubleProperty("frontRightAngleMotor", () -> frontRightAngleMotor.get(), null);		
		builder.addDoubleProperty("frontLeftSAngleMotor", () -> frontLeftAngleMotor.get(), null);
		builder.addDoubleProperty("backRightAngleMotor",  () -> backRightAngleMotor.get(),  null);
		builder.addDoubleProperty("backLeftAngleMotor",   () -> backLeftAngleMotor.get(),   null);
		
	}

   
}
