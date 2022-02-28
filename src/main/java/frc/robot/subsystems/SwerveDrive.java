package frc.robot.subsystems;

import java.util.ResourceBundle.Control;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.Encoder;
import frc.robot.RobotBase;

public class SwerveDrive extends SubsystemBase {
    
    private ADIS16470_IMU gyro;

    private double KpAnglefl = 0.01;
	private double KiAnglefl = 0;
	private double KdAnglefl = 0;

    private double KpAnglefr = 0.01;
	private double KiAnglefr = 0;
	private double KdAnglefr = 0;

    private double KpAnglebl = 0.01;
	private double KiAnglebl = 0;
	private double KdAnglebl = 0;

    private double KpAnglebr = 0.01;
	private double KiAnglebr = 0;
	private double KdAnglebr = 0;


    private WPI_TalonFX frontRightSpeedMotor;
    private WPI_TalonFX frontLeftSpeedMotor;
    private WPI_TalonFX backRightSpeedMotor;
    private WPI_TalonFX backLeftSpeedMotor;
    
    private WPI_TalonSRX frontRightAngleMotor;
    private WPI_TalonSRX frontLeftAngleMotor;
    private WPI_TalonSRX backRightAngleMotor;
    private WPI_TalonSRX backLeftAngleMotor;

    public boolean isfieldOriented = true;

    // private Encoder frontLeftEncoder;
    // private Encoder frontRightEncoder;
    // private Encoder backLeftEncoder;
    // private Encoder backRightEncoder;

    private PIDController pidAnglefl;
    private PIDController pidAnglefr;
    private PIDController pidAnglebl;
    private PIDController pidAnglebr;

    private final double L  = 23; //vehicle tracklength
    private final double W  = 23; //vehicle trackwidth
    private final double R  = Math.sqrt(Math.pow(L, 2) + Math.pow(W, 2));
    private final double PI = Math.PI; 
    
    private double positionAlongField = 0;
    private double positionAcrossField = 0;

    private NetworkTableEntry safeModeEntry;
    private NetworkTableEntry isFieldOriented;

    public enum Axis {FORWARD, STRAFE, TURN, BUTTON}
    public enum DriveMode {SAFEMMODE, FIELDMODE, GOALMODE, BALLMODE}
    public enum SwerveModule {FRONT_LEFT, FRONT_RIGHT, BACK_LEFT, BACK_RIGHT}



    public SwerveDrive(RobotBase robot) {
        
        super(robot);
        configMotors();

        //setToZeroPosition();
        configurePID();

        //gyro = new ADIS16470_IMU();

        reset();        
    }

    private void configMotors() {
        
        // frontRightSpeedMotor = new WPI_TalonSRX(port("frontRightSpeedMotor"));
        // frontLeftSpeedMotor  = new WPI_TalonSRX(port("frontLeftSpeedMotor"));
        // backRightSpeedMotor  = new WPI_TalonSRX(port("backRightSpeedMotor"));
        // backLeftSpeedMotor   = new WPI_TalonSRX(port("backLeftSpeedMotor"));


        frontRightSpeedMotor = new WPI_TalonFX(port("frontRightSpeedMotor"));
        frontLeftSpeedMotor  = new WPI_TalonFX(port("frontLeftSpeedMotor"));
        backRightSpeedMotor  = new WPI_TalonFX(port("backRightSpeedMotor"));
        backLeftSpeedMotor   = new WPI_TalonFX(port("backLeftSpeedMotor"));

        
        frontRightAngleMotor = new WPI_TalonSRX(port("frontRightAngleMotor"));
        frontLeftAngleMotor  = new WPI_TalonSRX(port("frontLeftSAngleMotor"));
        backRightAngleMotor  = new WPI_TalonSRX(port("backRightAngleMotor"));
        backLeftAngleMotor   = new WPI_TalonSRX(port("backLeftAngleMotor"));

        

        addChild("frontRightSpeedMotor", frontRightSpeedMotor);
        addChild("frontLeftSpeedMotor",  frontLeftSpeedMotor);
        addChild("backRightSpeedMotor",  backRightSpeedMotor);
        addChild("backLeftSpeedMotor",   backLeftSpeedMotor);

        addChild("frontRightAngleMotor", frontRightAngleMotor);
        addChild("frontLeftSAngleMotor", frontLeftAngleMotor);
        addChild("backRightAngleMotor",  backRightAngleMotor);
        addChild("backLeftAngleMotor",   backLeftAngleMotor);

        //invert speed motors
        frontRightSpeedMotor.setInverted(true);
        frontLeftSpeedMotor.setInverted(false);
        backRightSpeedMotor.setInverted(true);
        backLeftSpeedMotor.setInverted(false);

        // testMotor.set(TalonFXControlMode.Position, pidAnglebl.calculate(testMotor.getSelectedSensorPosition(), 0));
        // testMotor.getSelectedSensorPosition();

        //invert angle motors
        frontRightAngleMotor.setInverted(false);
        frontLeftAngleMotor.setInverted(true);
        backRightAngleMotor.setInverted(true);
        backLeftAngleMotor.setInverted(true);

        frontRightAngleMotor.configFactoryDefault();
        frontLeftAngleMotor.configFactoryDefault();
        backRightAngleMotor.configFactoryDefault();
        backLeftAngleMotor.configFactoryDefault();
    
        frontRightAngleMotor.configSelectedFeedbackSensor(FeedbackDevice.Analog);
        frontLeftAngleMotor.configSelectedFeedbackSensor(FeedbackDevice.Analog);
        backRightAngleMotor.configSelectedFeedbackSensor(FeedbackDevice.Analog);
        backLeftAngleMotor.configSelectedFeedbackSensor(FeedbackDevice.Analog);

        frontRightAngleMotor.setSensorPhase(true);
        frontLeftAngleMotor.setSensorPhase(false);
        backLeftAngleMotor.setSensorPhase(false);
        backRightAngleMotor.setSensorPhase(false);

        //frontLeftAngleMotor.configFeedbackNotContinuous(false, 30);

       
		// int absolutePositionfl = frontLeftAngleMotor.getSensorCollection().getPulseWidthPosition();
		// int absolutePositionfr = frontRightAngleMotor.getSensorCollection().getPulseWidthPosition();
		// int absolutePositionbr = backRightAngleMotor.getSensorCollection().getPulseWidthPosition();
		// int absolutePositionbl = backLeftAngleMotor.getSensorCollection().getPulseWidthPosition();

		// // Mask out overflows, keep bottom 12 bits
        // absolutePositionfl &= 0xFFF;
		// absolutePositionfr &= 0xFFF;
		// absolutePositionbr &= 0xFFF;
		// absolutePositionbl &= 0xFFF;

        // if (frontLeftAngleMotor.getInverted())  { absolutePositionfl *= -1; }
		// if (frontRightAngleMotor.getInverted()) { absolutePositionfr *= -1; }
		// if (backRightAngleMotor.getInverted())  { absolutePositionbr *= -1; }
		// if (backLeftAngleMotor.getInverted())   { absolutePositionbl *= -1; }

        // if (false) { absolutePositionfl *= -1; }
        // if (true)  { absolutePositionfr *= -1; }
        // if (false) { absolutePositionbr *= -1; }
        // if (false) { absolutePositionbl *= -1; }
		
		// /* Set the quadrature (relative) sensor to match absolute */
		// frontLeftAngleMotor.setSelectedSensorPosition(absolutePositionfl);
		// frontRightAngleMotor.setSelectedSensorPosition(absolutePositionfr);
		// backRightAngleMotor.setSelectedSensorPosition(absolutePositionbr);
		// backLeftAngleMotor.setSelectedSensorPosition(absolutePositionbl);
        
        frontRightSpeedMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
        frontLeftSpeedMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
        backRightSpeedMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
        backLeftSpeedMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);



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

    private void configurePID() {

        pidAnglefl = new PIDController(KpAnglefl, KiAnglefl, KdAnglefl);
        pidAnglefr = new PIDController(KpAnglefr, KiAnglefr, KdAnglefr);
        pidAnglebl = new PIDController(KpAnglebl, KiAnglebl, KdAnglebl);
        pidAnglebr = new PIDController(KpAnglebr, KiAnglebr, KdAnglebr);
        

	}

    // private void configureEncoders() {

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
    public void calculateDrive(double FWD, double STR, double RCW, double gryroAngle, boolean driveMode) {
               
        if(driveMode) {
            
            double temp = FWD*Math.cos(gryroAngle) + STR*Math.sin(gryroAngle);
        
            STR = -FWD*Math.sin(gryroAngle) + STR*Math.cos(gryroAngle);
            FWD = temp;

        } else {
            
            double temp = FWD*Math.cos(0) + STR*Math.sin(0);
        
            STR = -FWD*Math.sin(0) + STR*Math.cos(0);
            FWD = temp;

        }

        double A = STR - RCW*(L/R);
        double B = STR + RCW*(L/R);
        double C = FWD - RCW*(W/R);
        double D = FWD + RCW*(W/R); 

        double frontRightWheelSpeed = Math.sqrt(Math.pow(B, 2) + Math.pow(C, 2));
        double frontLeftWheelSpeed  = Math.sqrt(Math.pow(B, 2) + Math.pow(D, 2));
        double backLeftWheelSpeed   = Math.sqrt(Math.pow(A, 2) + Math.pow(D, 2));
        double backRightWheelSpeed  = Math.sqrt(Math.pow(A, 2) + Math.pow(C, 2));

        double frontRightWheelAngle = (Math.atan2(B, C) * (180 / PI));
        double frontLeftWheelAngle  = (Math.atan2(B, D) * (180 / PI));//
        double backLeftWheelAngle   = (Math.atan2(A, D) * (180 / PI));
        double backRightWheelAngle  = (Math.atan2(A, C) * (180 / PI));//

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

        frontRightWheelSpeed = max; 
        frontLeftWheelSpeed  = max; 
        backLeftWheelSpeed   = max; 
        backRightWheelSpeed  = max;

        // if (Math.abs(frontRightWheelAngle - ticksToDegrees(frontRightAngleMotor)) > 90 && Math.abs(frontRightWheelAngle - ticksToDegrees(frontRightAngleMotor)) < 270) {
        //     frontRightWheelAngle = (((int)frontRightWheelAngle + 180) % 360);
        //     frontRightWheelSpeed = -frontRightWheelSpeed;
        // }

        // if (Math.abs(frontLeftWheelAngle - ticksToDegrees(frontLeftAngleMotor)) > 90 && Math.abs(frontLeftWheelAngle - ticksToDegrees(frontLeftAngleMotor)) < 270) {
        //     frontLeftWheelAngle = (((int)frontLeftWheelAngle + 180) % 360);
        //     frontLeftWheelSpeed = -frontLeftWheelSpeed;
        // }

        // if (Math.abs(backLeftWheelAngle - ticksToDegrees(backLeftAngleMotor)) > 90 && Math.abs(backLeftWheelAngle - ticksToDegrees(backLeftAngleMotor)) < 270) {
        //     backLeftWheelAngle = ((int)backLeftWheelAngle + 180) % 360;
        //     backLeftWheelSpeed = -backLeftWheelSpeed;
        // }

        // if (Math.abs(backRightWheelAngle - ticksToDegrees(backRightAngleMotor)) > 90 && Math.abs(backRightWheelAngle - ticksToDegrees(backRightAngleMotor)) < 270) {
        //     backRightWheelAngle = ((int)backRightWheelAngle + 180) % 360;
        //     backRightWheelSpeed = -backRightWheelSpeed;
        // }

        // frontRightSpeedMotor.set(frontRightWheelSpeed);
        // frontLeftSpeedMotor.set(frontLeftWheelSpeed);
        // backRightSpeedMotor.set(backLeftWheelSpeed);
      // backLeftSpeedMotor.set(backRightWheelSpeed);
        backLeftAngleMotor.set(pidAnglebl.calculate(ticksToDegrees(backLeftAngleMotor),      setDirection(backLeftWheelAngle,   backLeftAngleMotor, backLeftSpeedMotor, backLeftWheelSpeed)));
       backRightAngleMotor.set(pidAnglebr.calculate(ticksToDegrees(backRightAngleMotor),   setDirection(backRightWheelAngle,  backRightAngleMotor, backRightSpeedMotor, backRightWheelSpeed)));
        frontRightAngleMotor.set(pidAnglefr.calculate(ticksToDegrees(frontRightAngleMotor), setDirection(frontRightWheelAngle, frontRightAngleMotor, frontRightSpeedMotor, frontRightWheelSpeed)));
        frontLeftAngleMotor.set(pidAnglefl.calculate(ticksToDegrees(frontLeftAngleMotor),   setDirection(frontLeftWheelAngle,  frontLeftAngleMotor, frontLeftSpeedMotor, frontLeftWheelSpeed)));
    }

    /**
 * Get the closest angle between the given angles.
 */
    private static double closestAngle(double a, double b)
    {
            // get direction
            double dir = (b % 360) - (a % 360);

            // convert from -360 to 360 to -180 to 180
            if (Math.abs(dir) > 180.0)
            {
                    dir = -(Math.signum(dir) * 360.0) + dir;
            }
            return dir;
    }

    // public double setDirection(double setpoint, WPI_TalonSRX motor)
    //     {
    //         //directionController.reset();

    //         // use the fastest way
    //         double currentAngle = ticksToDegrees(motor);
            
    //         return currentAngle + closestAngle(currentAngle, setpoint);
    //     }

        public double setDirection(double setpoint, WPI_TalonSRX motor, WPI_TalonFX speedMotor, double speed)
        {
            
            double direction = 0;
            double currentAngle = ticksToDegrees(motor);
            // find closest angle to setpoint
            double setpointAngle = closestAngle(currentAngle, setpoint);
            // find closest angle to setpoint + 180
            double setpointAngleFlipped = closestAngle(currentAngle, setpoint + 180.0);
            // if the closest angle to setpoint is shorter
            if (Math.abs(setpointAngle) <= Math.abs(setpointAngleFlipped))
            {
                // unflip the motor direction use the setpoint
                direction = currentAngle + setpointAngle;
            }
            // if the closest angle to setpoint + 180 is shorter
            else
            {
                // flip the motor direction and use the setpoint + 180
                speed = -speed;
                direction = currentAngle + setpointAngleFlipped;
            }
    
            speedMotor.set(speed);
            return direction;
        }
    

           
    private double integral, previous_error = 0;

    private double PID(double setpoint, WPI_TalonSRX motor){
        
        double P = 0.007;
        double I = 0;
        double D = 0;

        double deadzone = 2;
        
        double error = setpoint - ticksToDegrees(motor); // Error = Target - Actual
        integral += (error*.02); // Integral is increased by the error*time (which is .02 seconds using normal IterativeRobot)
        double derivative = (error - previous_error) / .02;
        
        double correction = P * error + I * integral + D * derivative;
        
        if(Math.abs(error) < deadzone)
            correction = 0;

        
        return correction;
    }

    private double ticksToDegrees(WPI_TalonSRX motor) {
        return motor.getSelectedSensorPosition() / (2.844444444444444444444444444444444444444444444);
    }
    
    private double calcYawStraight(double targetAngle, double currentAngle, double kP) {
        double errorAngle = (targetAngle - currentAngle) % 360;
        double correction = errorAngle * kP;

        return correction;
    }

    public double correctHeading(double currentAngle, double kP, double FWD, double STR, double RCW) {
        
        double storedHeading = 0;
        double correction = 0;

        if(RCW != 0) {
            storedHeading = currentAngle;
        } else {

            if(Math.abs(FWD) > 0 || Math.abs(STR) > 0) {
                correction = calcYawStraight(storedHeading, currentAngle, kP);
            }
        }

        return correction;
    }
    
    //TODO: need to test this
    private void setSpeedAndAngle(Encoder encoder, WPI_TalonSRX angleMotor, WPI_TalonSRX speedMotor, double calculatedAngle, double speed, PIDController pidController) { 
        
        double joystickAngle = calculatedAngle;
        
        //Step 1: get current encoder raw
        double encoderRaw = encoder.getDistance();

        //Step 2: convert raw value to 360 scale
        double encoder360Scale = encoderRaw % 360;


        //Step 3: find the closest angle in 360 scale
        if (Math.abs(joystickAngle - encoder360Scale) > 90 && Math.abs(joystickAngle - encoder360Scale) < 270) {
            joystickAngle = ((int)joystickAngle + 180) % 360;
            speed = -speed; //invert the motors
        }

        //Steb 3b:if the wrapped value is over 270 we have to add 360 to the joystrick angle to
        // see if it is closest
        if(joystickAngle > 270) {
            
            joystickAngle = joystickAngle + 360;
        
        } else if(joystickAngle < 90) { //same thing just the opposite direction
            
            joystickAngle = joystickAngle - 360;
        
        }

        //Step 4: difference = (joystickAngle - current 360 angle)
        double difference = joystickAngle - encoder360Scale;

        //Step 5: Final raw destination angle = current encoder raw value + difference
        double finalDestination = encoderRaw + difference;

        if(MathUtil.applyDeadband(axis("strafe"),  0.1) == 0) {
            finalDestination = finalDestination + 360;
        }

        //Step 6: Feed the output of step 5 and the destination angle for the angle motor
        angleMotor.set(pidController.calculate(encoder.getDistance(), finalDestination));
        System.out.println("final destination: " + finalDestination);
        


        //Step 7: Set Wheel Speed
        speedMotor.set(speed);
       

    }

    // public void calculateRobotPosition() {
        
    //     double Bfl = Math.sin(frontLeftAngleMotor.getSelectedSensorPosition())  * frontLeftSpeedMotor.getSelectedSensorVelocity();
    //     double Bfr = Math.sin(frontRightAngleMotor.getSelectedSensorPosition()) * frontRightSpeedMotor.getSelectedSensorVelocity();
    //     double Abl = Math.sin(backLeftAngleMotor.getSelectedSensorPosition())   * backLeftSpeedMotor.getSelectedSensorVelocity();
    //     double Abr = Math.sin(backRightAngleMotor.getSelectedSensorPosition())  * backRightSpeedMotor.getSelectedSensorVelocity();

    //     double Dfl = Math.cos(frontLeftAngleMotor.getSelectedSensorPosition())  * frontLeftSpeedMotor.getSelectedSensorVelocity();
    //     double Cfr = Math.cos(frontRightAngleMotor.getSelectedSensorPosition()) * frontRightSpeedMotor.getSelectedSensorVelocity();
    //     double Dbl = Math.cos(backLeftAngleMotor.getSelectedSensorPosition())   * backLeftSpeedMotor.getSelectedSensorVelocity();
    //     double Cbr = Math.cos(backRightAngleMotor.getSelectedSensorPosition())  * backRightSpeedMotor.getSelectedSensorVelocity();

    //     double A = (Abr + Abl) / 2;
    //     double B = (Bfl + Bfr) / 2;
    //     double C = (Cfr + Cbr) / 2;
    //     double D = (Dfl + Dbl) / 2;

    //     double rotation1 = (B - A) / L;
    //     double rotation2 = (C - D) / W;
    //     double rotation = (rotation1  + rotation2) / 2;

    //     double forward1 = rotation * (L / 2) + A;
    //     double forward2 = -rotation * (L / 2) + B;
    //     double forward = (forward1 + forward2) / 2;

    //     double strafe1 = rotation * (W / 2) + C;
    //     double strafe2 = -rotation * ( W / 2) + D;
    //     double strafe = (strafe1 + strafe2) / 2;


    //     double forwardNew = (forward * Math.cos(gyro.getAngle())) + (strafe *  Math.sin(gyro.getAngle())); 
    //     double strafeNew  = (strafe *  Math.cos(gyro.getAngle())) - (forward * Math.sin(gyro.getAngle()));

    //     double timeStep = 0.020; //milliseconds //Timer.getFPGATimestamp() - lastTime //don't know how to do this so just constant from whitepaper
        
    //     positionAlongField = positionAlongField + (forwardNew * timeStep);
    //     positionAcrossField = positionAcrossField + (strafeNew * timeStep);

    // }

    private double[] getRobotPosition() {

        double[] coordinates = {positionAcrossField, positionAlongField};
        return coordinates;
    }

    public void resetRobotPosition() {
        positionAlongField = 0;
        positionAcrossField = 0;
    }
    //Drive methods for auto
    public void driveForward(double speed) {
        
        calculateDrive(speed, 0, 0, gyro.getAngle(), getIsFieldOriented());
    
    }


    public void driveBackward(double speed) {

        calculateDrive(-speed, 0, 0, gyro.getAngle(), getIsFieldOriented());
    }

    public void strafeLeft(double speed) {

        calculateDrive(0, speed, 0, gyro.getAngle(), getIsFieldOriented());

    }

    public void strafeRight(double speed) {

        calculateDrive(0, -speed, 0, gyro.getAngle(), getIsFieldOriented());

    }

    public void rotate(double speed) {

        calculateDrive(0, 0, speed, gyro.getAngle(), getIsFieldOriented());

    }

    public void swerveDrive(double FWD, double STR, double RCW, boolean isFieldOriented) {

        calculateDrive(FWD, STR, RCW, gyro.getAngle(), getIsFieldOriented());
    }

    public void halt() {
        
        calculateDrive(0, 0, 0, gyro.getAngle(), getIsFieldOriented());

    }

    public boolean getIsFieldOriented() {
        return isFieldOriented.getBoolean(false);
    }

    public void setIsFieldOriented(boolean enabled) {
        isFieldOriented.setBoolean(enabled);
    }
    
    public boolean getSafeMode() {
        return safeModeEntry.getBoolean(false);
    }

    public void setSafeMode(boolean enabled) {
        safeModeEntry.setBoolean(enabled);
    }

	@Override
	public void periodic() {
		
	}

	public void stop() {    

	}

	public void reset() {
		
	}
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

		// builder.addDoubleProperty("Forward", () ->   MathUtil.applyDeadband(axis("forward"), DEADBAND), null);
		// builder.addDoubleProperty("Strafe",  () ->  -MathUtil.applyDeadband(axis("strafe"),  DEADBAND), null);
		// builder.addDoubleProperty("Rotate",  () ->   MathUtil.applyDeadband(axis("rotate"),  DEADBAND), null);
        
        builder.addDoubleProperty("Front Right Angle", () -> ticksToDegrees(frontRightAngleMotor), null);		
		builder.addDoubleProperty("Front Left Angle",  () -> ticksToDegrees(frontLeftAngleMotor),  null);
		builder.addDoubleProperty("Back Right Angle",  () -> ticksToDegrees(backRightAngleMotor),  null);
		builder.addDoubleProperty("Back Left Angle",   () -> ticksToDegrees(backLeftAngleMotor),   null);

       // builder.addDoubleProperty("frontLeftEncoer", () -> frontRightSpeedMotor.getSelectedSensorPosition(), null);

        builder.addDoubleArrayProperty("Robot Position", () -> getRobotPosition(), null);

        // builder.addBooleanProperty("isSafeMode", () -> safeMode, null);
        // builder.addBooleanProperty("Drive Mode", () -> getIsFieldOriented(), null);
		
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
