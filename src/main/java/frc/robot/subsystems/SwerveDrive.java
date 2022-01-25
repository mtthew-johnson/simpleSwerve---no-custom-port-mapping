package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotBase;

public class SwerveDrive extends SubsystemBase {

    private static final int lastTime = 0;

    private final double SPEED = 0.4;

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
    private WPI_TalonSRX frontLeftAngleMotor;
    private WPI_TalonSRX backRightAngleMotor;
    private WPI_TalonSRX backLeftAngleMotor;

    private boolean safeMode = false;

    private double forwardtemp = axis("forward");
    private double strafetemp = axis("strafe");
    private double rotatetemp = axis("rotate");

    private ADXRS450_Gyro gyro;

    private Encoder frontLeftEncoder;
    private Encoder frontRightEncoder;
    private Encoder backLeftEncoder;
    private Encoder backRightEncoder;

    private EncoderSim frontLeftEncoderSim;
    private EncoderSim frontRightEncoderSim;
    private EncoderSim backLeftEncoderSim;
    private EncoderSim backRightEncoderSim;

    private PIDController pidAnglefl;
    private PIDController pidAnglefr;
    private PIDController pidAnglebl;
    private PIDController pidAnglebr;

    private final double L = 23; //vehicle tracklength
    private final double W = 23; //vehicle trackwidth
    private final double R = Math.sqrt(Math.pow(L, 2) + Math.pow(W, 2));
    private final double PI = Math.PI; 
    
    private double positionAlongField = 0;
    private double positionAcrossField = 0;

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
        frontLeftAngleMotor = new WPI_TalonSRX(port("frontLeftSAngleMotor"));
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

        frontRightSpeedMotor.setInverted(true);
        frontLeftSpeedMotor.setInverted(true);
        backRightSpeedMotor.setInverted(false);
        backLeftSpeedMotor.setInverted(true);

        frontRightAngleMotor.setInverted(false);
        frontLeftAngleMotor.setInverted(false);
        backRightAngleMotor.setInverted(false);
        backLeftAngleMotor.setInverted(true);
    
        frontRightAngleMotor.configSelectedFeedbackSensor(FeedbackDevice.Analog);
        frontLeftAngleMotor.configSelectedFeedbackSensor(FeedbackDevice.Analog);
        backRightAngleMotor.configSelectedFeedbackSensor(FeedbackDevice.Analog);
        backLeftAngleMotor.configSelectedFeedbackSensor(FeedbackDevice.Analog);
        
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
 
    private void calculateDrive(double FWD, double STR, double RCW, double gryroAngle) {
       
        double temp = FWD*Math.cos(gryroAngle) + STR*Math.sin(gryroAngle);
        
        STR = -FWD*Math.sin(gryroAngle) + STR*Math.cos(gryroAngle);
        FWD = temp;

        double A = STR - RCW*(L/R);
        double B = STR + RCW*(L/R);
        double C = FWD - RCW*(W/R);
        double D = FWD + RCW*(W/R); 

        double frontRightWheelSpeed = Math.sqrt(Math.pow(B, 2) + Math.pow(C, 2));
        double frontLeftWheelSpeed  = Math.sqrt(Math.pow(B, 2) + Math.pow(D, 2));
        double backLeftWheelSpeed   = Math.sqrt(Math.pow(A, 2) + Math.pow(D, 2));
        double backRightWheelSpeed  = Math.sqrt(Math.pow(A, 2) + Math.pow(C, 2));

        double frontRightWheelAngle = (Math.atan2(B, C) * (180 / PI));
        double frontLeftWheelAngle  = -(Math.atan2(B, D) * (180 / PI));
        double backLeftWheelAngle   = (Math.atan2(A, D) * (180 / PI));
        double backRightWheelAngle  = -(Math.atan2(A, C) * (180 / PI));

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

        frontRightWheelSpeed = (max * SPEED); 
        frontLeftWheelSpeed  = (max * SPEED); 
        backLeftWheelSpeed   = (max * SPEED); 
        backRightWheelSpeed  = (max * SPEED);

        frontRightSpeedMotor.set(frontRightWheelSpeed);
        frontLeftSpeedMotor.set(frontLeftWheelSpeed);
        backRightSpeedMotor.set(backLeftWheelSpeed);
        backLeftSpeedMotor.set(backRightWheelSpeed);

        
        // backLeftAngleMotor.set(pidAnglebl.calculate(backLeftEncoder.getDistance(), backLeftWheelAngle));
        // backRightAngleMotor.set(pidAnglebr.calculate(backRightEncoder.getDistance(), backRightWheelAngle));
        // frontRightAngleMotor.set(pidAnglefr.calculate(frontRightEncoder.getDistance(), frontRightWheelAngle));
        // frontLeftAngleMotor.set(pidAnglefl.calculate(frontLeftEncoder.getDistance(), frontLeftWheelAngle));

        setOptmizedAngle(backLeftEncoder,   backLeftAngleMotor,   backLeftSpeedMotor,   pidAnglebl, backLeftWheelAngle, true);//
        setOptmizedAngle(backRightEncoder,  backRightAngleMotor,  backRightSpeedMotor,  pidAnglebr, backRightWheelAngle, false);
        setOptmizedAngle(frontRightEncoder, frontRightAngleMotor, frontRightSpeedMotor, pidAnglefr, frontRightWheelAngle, true);//
        setOptmizedAngle(frontLeftEncoder,  frontLeftAngleMotor,  frontLeftSpeedMotor,  pidAnglefl, frontLeftWheelAngle, true);

        //frontright
        //backleft
       
        // resetAfterFullRotation(backLeftEncoder);
        // resetAfterFullRotation(backRightEncoder);
        // resetAfterFullRotation(frontRightEncoder);
        // resetAfterFullRotation(frontLeftEncoder);
        
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

        frontRightEncoderSim = new EncoderSim(frontRightEncoder);
        frontLeftEncoderSim  = new EncoderSim(frontLeftEncoder);
        backRightEncoderSim  = new EncoderSim(backRightEncoder);
        backLeftEncoderSim   = new EncoderSim(backLeftEncoder);
    }

    private void resetAfterFullRotation(Encoder encoder) {                                                   //left     /right
        //again, assuming 0-360 degrees is one rotation to the left/right //TODO: need figure out which way is positive/negative
        //               -360-0 degress is one rotation to the left/right idk
        if(encoder.getDistance() > 360 || encoder.getDistance() < -360) {
           
            encoder.reset();

        }
    }

    //TODO: need to test this
    private void setOptmizedAngle(Encoder encoder, WPI_TalonSRX angleMotor, WPI_TalonSRX speedMotor, PIDController pidController, double targetAngle, boolean initialInvert) { 
        //potential angle optimization - assuming 0-360 degrees of motion
        //I still don't really know the exact outputs for the angle calculations/exactly how they work
        //The idea is that based on the current angle of the wheel, we could just figure out the closest angle and just invert the motor
        //simple example:
        //when using the controller for the robot go forwards, the wheels are facing the 'zero' angle and then the speed motors get positive power
        //then to backwards the robot will currently rotate all the wheels 180 degrees then apply positive power to go backwards
        //so if we knew the current angle of the motors was 0 (forward) and wanted to go backwards, we just invert the motors - which would be more efficient

        //a basic attempt at this

        // if(encoder.getDistance() == 0 && axis("forward") < 0) {
        //     if(speedMotor.getInverted() == true) {
               
        //         speedMotor.setInverted(false);
                
        //     } else {

        //         speedMotor.setInverted(true);
        //     }
        // }

        //now how would we apply this to all possible angles?
        // if(encoder.getDistance() == (targetAngle - 180) || encoder.getDistance() == (targetAngle + 180)) {
                        
        //     if(speedMotor.getInverted() == true) {
               
        //         speedMotor.setInverted(false);
        //         angleMotor.set(pidController.calculate(encoder.getDistance(), encoder.getDistance())); //invert motor speed but keep the angle the same
                
        //     } else {

        //         speedMotor.setInverted(true);
        //         angleMotor.set(pidController.calculate(encoder.getDistance(), encoder.getDistance())); //invert motor speed but keep the angle the same

        //     }
        // }

        //now the above logic only works if the current angle is exactly opposite from the target angle
        //so how to move the angle motor to the closest setpoint (then invert motors if need be)?

        //psuedo code
        // if (current motor angle is closer to calculated angle or is it closer to the calculated angle - 180 or + 180)
            // invert motor
            // set motor angle
      //tolerance is within 1 degree - should be good enough - adjust as necessary
        final int TOLERANCE_UPPER = 183;
        final int TOLERANCE_LOWER = 177;
        //still don't know what the positive/negative direction for the encoders are, so the inverting may need to be adjusted
        //but it actually might not as it will only invert if the angle is set to an 'optmized one'
        boolean isInRange = (Math.abs(encoder.getDistance()) > (TOLERANCE_LOWER - targetAngle) && Math.abs(encoder.getDistance()) < (targetAngle - TOLERANCE_UPPER)) ||
                            (Math.abs(encoder.getDistance()) > (TOLERANCE_LOWER + targetAngle) && Math.abs(encoder.getDistance()) < (targetAngle + TOLERANCE_UPPER));


        // final boolean initalfr = false;
        // final boolean initialfl = false;
        // final boolean initialbr = false;
        // final boolean initialbl = true;
                        
        //given values a, b, c, d and c is the current value
        //'a' being the lower bound (targetAngle - 180)
        //'b' being the current target angle
        //'c' being the current wheel angle
        //'d' being the upper bound (targetAngle + 180)
        //try to determine whether a, b, or d is closest to c
        boolean flag = true;
        if(Math.abs(encoder.getDistance() - (Math.abs(targetAngle - 180))) < Math.abs(encoder.getDistance() - (Math.abs(targetAngle + 180)))) { //a is closer to c then d
            
            if(Math.abs(encoder.getDistance() - (Math.abs(targetAngle - 180))) < Math.abs(encoder.getDistance() - targetAngle)) { //a is closer to c then b
                
                angleMotor.set(pidController.calculate(encoder.getDistance(), targetAngle - 180));
                
                //now that the angle is set, now we need to determine whether to invert the motors or not
                if(isInRange) {
                    
                    if(initialInvert == true) {
               
                        speedMotor.setInverted(false);
                        
                    } else if(initialInvert == false) {
        
                        speedMotor.setInverted(true);
        
                    }
                }

            } else { // b is closer to c then a
                
                angleMotor.set(pidController.calculate(encoder.getDistance(), targetAngle));

                speedMotor.setInverted(initialInvert);

                 //now that the angle is set, now we need to determine whether to invert the motors or not
        
            }

        } else { //d is closer to c then a
           
            if(Math.abs(encoder.getDistance() - (Math.abs(targetAngle + 180))) < Math.abs(encoder.getDistance() - targetAngle)) { //d is closer to c then b

                angleMotor.set(pidController.calculate(encoder.getDistance(), targetAngle + 180));
                
                //now that the angle is set, now we need to determine whether to invert the motors or not
                if(isInRange) {
                    
                    if(initialInvert == true) {
               
                        speedMotor.setInverted(false);
                        
                    } else if(initialInvert == false) {
        
                        speedMotor.setInverted(true);
        
                    }
                }

            } else { //b is closer to c then a
                
                angleMotor.set(pidController.calculate(encoder.getDistance(), targetAngle));
                speedMotor.setInverted(initialInvert);

                 //now that the angle is set, now we need to determine whether to invert the motors or not
              
            }
        }
    }

    private boolean isOptmized(Encoder encoder) {
        //TODO: there might possibly be issues with calculating odometry if wheel angles are optmized
        //TODO: so we need to figure out a way to know if the angle motors are using the optimized angle and fix the math for odometry
        return true;
    }

    private void calculateRobotPosition() {
        
        double Bfl = Math.sin(frontLeftEncoder.getDistance())  * frontLeftSpeedMotor.getSelectedSensorVelocity();
        double Bfr = Math.sin(frontRightEncoder.getDistance()) * frontRightSpeedMotor.getSelectedSensorVelocity();
        double Abl = Math.sin(backLeftEncoder.getDistance()) * backLeftSpeedMotor.getSelectedSensorVelocity();
        double Abr = Math.sin(backRightEncoder.getDistance()) * backRightSpeedMotor.getSelectedSensorVelocity();

        double Dfl = Math.cos(frontLeftEncoder.getDistance()) * frontLeftSpeedMotor.getSelectedSensorVelocity();
        double Cfr = Math.cos(frontRightEncoder.getDistance()) * frontRightSpeedMotor.getSelectedSensorVelocity();
        double Dbl = Math.cos(backLeftEncoder.getDistance()) * backLeftSpeedMotor.getSelectedSensorVelocity();
        double Cbr = Math.cos(backRightEncoder.getDistance()) * backRightSpeedMotor.getSelectedSensorVelocity();

        double A = (Abr + Abl) / 2;
        double B = (Bfl + Bfr) / 2;
        double C = (Cfr + Cbr) / 2;
        double D = (Dfl + Dbl) / 2;

        double rotation1 = (B - A) / L;
        double rotation2 = (C - D) / W;
        double rotation = (rotation1  + rotation2) / 2;

        double forward1 = rotation * (L / 2) + A;
        double forward2 = -rotation * (L / 2) + B;
        double forward = (forward1 + forward2) / 2;

        double strafe1 = rotation * (W / 2) + C;
        double strafe2 = -rotation * ( W / 2) + D;
        double strafe = (strafe1 + strafe2) / 2;


        double forwardNew = (forward * Math.cos(gyro.getAngle())) + (strafe *  Math.sin(gyro.getAngle())); 
        double strafeNew  = (strafe *  Math.cos(gyro.getAngle())) - (forward * Math.sin(gyro.getAngle()));

        double timeStep = 0.020; //milliseconds //Timer.getFPGATimestamp() - lastTime //don't know how to do this so just constant from whitepaper
        
        positionAlongField = positionAlongField + (forwardNew * timeStep);
        positionAcrossField = positionAcrossField + (strafeNew * timeStep);

    }

    private double[] getRobotPosition() {

        double[] coordinates = {positionAcrossField, positionAlongField};
        return coordinates;
    }

    private void resetRobotPosition() {
        positionAlongField = 0;
        positionAcrossField = 0;
    }

    private void rotateToAngle(double rotationSpeed, double gyroAngle, double angleToRotate) {
        
        while(!(Math.abs(gyroAngle) < Math.abs(angleToRotate - 1) && Math.abs(gyroAngle) > Math.abs(angleToRotate + 1))) {
            
            if(gyroAngle < angleToRotate) {
                
                calculateDrive(0, 0, -rotationSpeed, gyroAngle);

            } else if (gyroAngle > angleToRotate) {
                
                calculateDrive(0, 0, rotationSpeed, gyroAngle);

            }

        }
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

                //stop robot from drifting

                // if(Math.abs(rotatetemp) < 0.01) {
                    
                //     rotateToAngle(0.5, gyro.getAngle(), 0);

                // }

                // System.out.println("forwardtemp: " + forwardtemp);
                // System.out.println("strafetemp:  " + strafetemp);
                // System.out.println("rotatetemp:  " + rotatetemp);

                calculateDrive(forwardtemp, strafetemp, rotatetemp, gyro.getAngle());

                System.out.println("frontLeftAngle:  " + frontLeftEncoder.getDistance());
                System.out.println("frontRightAngle: " + frontRightEncoder.getDistance());
                System.out.println("backLeftAngle:   " + backLeftEncoder.getDistance());
                System.out.println("backRigthAngle:  " + backRightEncoder.getDistance());
                //calculateRobotPosition();

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

		builder.addDoubleProperty("Forward", () -> forwardtemp, (value) -> forwardtemp = value);
		builder.addDoubleProperty("Strafe",  () -> strafetemp,  (value) -> strafetemp = value);
		builder.addDoubleProperty("Rotate",  () -> rotatetemp,  (value) -> rotatetemp = value);
        
        builder.addDoubleProperty("Front Right Angle", () -> frontRightEncoder.getDistance(), null);		
		builder.addDoubleProperty("Front Left Angle",  () -> frontLeftEncoder.getDistance(),  null);
		builder.addDoubleProperty("Back Right Angle",  () -> backRightEncoder.getDistance(),  null);
		builder.addDoubleProperty("Back Left Angle",   () -> backLeftEncoder.getDistance(),   null);

        builder.addDoubleArrayProperty("Robot Position", () -> getRobotPosition(), null);

        builder.addBooleanProperty("isSafeMode", () -> safeMode, null);
		
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
