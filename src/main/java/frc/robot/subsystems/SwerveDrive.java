package frc.robot.subsystems;

import java.util.ResourceBundle.Control;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.hal.EncoderJNI;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Encoder;
import frc.robot.RobotBase;

public class SwerveDrive extends SubsystemBase {

    private final double SPEED = 0.7;
    
    private double Kp = 0.04;
	private double Ki = 0;
	private double Kd = 0.002;
	private double correction = 0;
	private double prevAngle = 0;
	private double angleCompensation = 0;

    private WPI_TalonSRX frontRightSpeedMotor;
    private WPI_TalonSRX frontLeftSpeedMotor;
    private WPI_TalonSRX backRightSpeedMotor;
    private WPI_TalonSRX backLeftSpeedMotor;
    
    private WPI_TalonSRX frontRightAngleMotor;
    private WPI_TalonSRX frontLeftSAngleMotor;
    private WPI_TalonSRX backRightAngleMotor;
    private WPI_TalonSRX backLeftAngleMotor;

    private boolean drivePOV = false;
    private boolean safeMode = false;

    public ADXRS450_Gyro gyro;

    


    public SwerveDrive(RobotBase robot) {
        super(robot);
       
        initMotors();
        configureGyro();
        
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

        addChild("frontRightSpeedMotor Motor", frontRightSpeedMotor);
        addChild("frontLeftSpeedMotor Motor",  frontLeftSpeedMotor);
        addChild("backRightSpeedMotor Motor",  backRightSpeedMotor);
        addChild("backLeftSpeedMotor Motor",   backLeftSpeedMotor);

        addChild("frontRightAngleMotor Motor", frontRightAngleMotor);
        addChild("frontLeftSAngleMotor Motor", frontLeftSAngleMotor);
        addChild("backRightAngleMotor Motor",  backRightAngleMotor);
        addChild("backLeftAngleMotor Motor",   backLeftAngleMotor);

        frontRightSpeedMotor.setInverted(false);
        frontLeftSpeedMotor.setInverted(false);
        backRightSpeedMotor.setInverted(false);
        backLeftSpeedMotor.setInverted(false);

        frontRightAngleMotor.setInverted(false);
        frontLeftSAngleMotor.setInverted(false);
        backRightAngleMotor.setInverted(false);
        backLeftAngleMotor.setInverted(false);
        
        frontRightSpeedMotor.set(ControlMode.Current, 0);
        frontLeftSpeedMotor.set(ControlMode.Current, 0);
        backRightSpeedMotor.set(ControlMode.Current, 0);
        backLeftSpeedMotor.set(ControlMode.Current, 0);
        
        frontRightAngleMotor.set(ControlMode.Position, 0);
        frontLeftSAngleMotor.set(ControlMode.Position, 0);
        backRightAngleMotor.set(ControlMode.Position, 0);
        backLeftAngleMotor.set(ControlMode.Position, 0);

    
        frontRightAngleMotor.configSelectedFeedbackSensor(FeedbackDevice.Analog);
        frontLeftSAngleMotor.configSelectedFeedbackSensor(FeedbackDevice.Analog);
        backRightAngleMotor.configSelectedFeedbackSensor(FeedbackDevice.Analog);
        backLeftAngleMotor.configSelectedFeedbackSensor(FeedbackDevice.Analog);
        
        frontRightSpeedMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
        frontLeftSpeedMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
        backRightSpeedMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
        backLeftSpeedMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
    
    
    }
    
    private void calculateDrive(double FWD, double STR, double RCW, double gryroAngle) {
       
        double temp = FWD*Math.cos(gryroAngle) + STR*Math.sin(gryroAngle);
        
        STR = -FWD*Math.sin(gryroAngle) + STR*Math.cos(gryroAngle);
        FWD = temp;

        final double PI = Math.PI;
        final double L = 0; //TODO find vehicle wheelbase
        final double W = 0; //TODO find vehicle trackwidth
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

        frontRightWheelSpeed = max * SPEED;
        frontLeftWheelSpeed  = max * SPEED;
        backLeftWheelSpeed   = max * SPEED;
        backRightWheelSpeed  = max * SPEED;

        //convert angle to talon ticks to set final angle/ 4096 ticks
        frontRightWheelAngle = ((frontRightWheelAngle + 180) * (4095 / 360));
        frontLeftWheelAngle  = ((frontLeftWheelAngle  + 180) * (4095 / 360));
        backLeftWheelAngle   = ((backLeftWheelAngle   + 180) * (4095 / 360));
        backRightWheelAngle  = ((backRightWheelAngle  + 180) * (4095 / 360));





    }


    
    private void configureGyro() {
		
		gyro = new ADXRS450_Gyro();
		gyro.reset();

	}

	@Override
	public void periodic() {
		
	}

	public void stop() {

	}

	public void reset() {
		
	}
    
}
