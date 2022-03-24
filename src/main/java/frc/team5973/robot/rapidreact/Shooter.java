package frc.team5973.robot.rapidreact;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.util.Color;
import frc.team5973.robot.Gains;
import frc.team5973.robot.RobotBase;
import frc.team5973.robot.subsystems.SubsystemBase;

public class Shooter extends SubsystemBase {
    
    private double SHOOTER_SPEED = 0.72;
    private final double OUTAKE_SPEED = 1;

    public WPI_TalonFX shooterWheel;
    private WPI_TalonSRX shooterOutake;

    private PIDController flywheelController = new PIDController(1, 0, 0);

    private ColorSensorV3 colorSensor;
    private ColorMatch   colorMatcher;

    private Color redBallColor;
    private Color blueBallColor;
                                    			     //   kP   	 kI    kD      kF          Iz    PeakOut */
    public final static Gains kGains_Velocit  = new Gains( 0.1, 0.001, 5, 1023.0/20660.0,  300,  1.00);

    private double motorOutput;

   // private DutyCycleEncoder rotateEncoder;

    public enum ShooterInput {BUTTON, SPIN_WHEEL}
    public enum ShooterAxis {INTERNAL_WHEEL}

    private final Timer timer = new Timer();
    
    public Shooter(RobotBase robot) {

        super(robot);

        configureMotors();
        //configureColorSensors();

    }

    private void configureMotors() {
        
        shooterWheel  = new WPI_TalonFX(port("shooterWheel"));
        shooterOutake = new WPI_TalonSRX(port("shooterOutake"));

        addChild("shooterWheel", shooterWheel);
        addChild("shooterOutake", shooterOutake);

        shooterWheel.configFactoryDefault();
        shooterOutake.configFactoryDefault();

        shooterWheel.configNeutralDeadband(0.001);

        
        shooterWheel.setInverted(true);
        shooterOutake.setInverted(true);

        shooterWheel.configOpenloopRamp(0);
        shooterWheel.configClosedloopRamp(0);

        shooterWheel.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 30);

/* Config the peak and nominal outputs */
        shooterWheel.configNominalOutputForward(0, 30);
        shooterWheel.configNominalOutputReverse(0, 30);
        shooterWheel.configPeakOutputForward(1, 30);
        shooterWheel.configPeakOutputReverse(-1, 30);  
        
        shooterWheel.config_kF(0, kGains_Velocit.kF,30);
		shooterWheel.config_kP(0, kGains_Velocit.kP, 30);
		shooterWheel.config_kI(0, kGains_Velocit.kI, 30);
		shooterWheel.config_kD(0, kGains_Velocit.kD,30);

        shooterOutake.configOpenloopRamp(0);
        shooterOutake.configClosedloopRamp(0);

        shooterOutake.setNeutralMode(NeutralMode.Brake);
    }

    // private void cofigEncoder() {
    //     rotateEncoder = new DutyCycleEncoder(0); //TODO: need ot figure out the correct port
    // }

    private void configureColorSensors() {
        
        colorSensor = new ColorSensorV3(Port.kMXP);
        colorMatcher = new ColorMatch();

        redBallColor  = new Color(0.143, 0.427, 0.429); //TODO need to calibrate these to detect the color of the balls
        blueBallColor = new Color(0.197, 0.561, 0.240);
        
    }

    public void shoot(double shooterSpeed) {
            
            //spin up flywheel
            shooterWheel.set(TalonFXControlMode.PercentOutput, shooterSpeed); //max velocity of flywheel 20000
            //System.out.println(shooterWheel.getSelectedSensorVelocity());

            
            // Wait for 1 second to allow wheel to spin up   
            timer.reset();
            timer.start();
            while (!timer.hasElapsed(1)) {}
            timer.stop();

            //send ball through
            shooterOutake.set(OUTAKE_SPEED);
    }

    public void halt() {
        shooterWheel.set(0);
        shooterOutake.set(0);
    }

    public void spinwheel(double speed) {
        shooterOutake.set(speed);
    }

    public void shootForTime(double time) {

        //spin up flywheel
        shooterWheel.set(SHOOTER_SPEED);
        
        // Wait for 1 second to allow wheel to spin up   
        timer.reset();
        timer.start();
        while (!timer.hasElapsed(1)) {}
        timer.stop();

        //send ball through
        shooterOutake.set(OUTAKE_SPEED);

        timer.reset();
        timer.start();
        while (!timer.hasElapsed(time)) {}
        timer.stop();

        shooterWheel.set(0);
        shooterOutake.set(0);

    }

    public boolean isBallObtianed() {
        
        boolean isRed  = detectedBallColor(colorSensor).equals("Red")  ? true : false;
        boolean isBlue = detectedBallColor(colorSensor).equals("Blue") ? true : false;
        
        return (isRed || isBlue) ? true : false;
    } 

    public boolean isBlueBallObtained() {
        return detectedBallColor(colorSensor).equals("Blue") ? true : false;
    }

    public boolean isRedBallObtained() {
        return detectedBallColor(colorSensor).equals("Red")  ? true : false;
    }

    public String detectedBallColor(ColorSensorV3 colorSensor) {
        
        String colorString;

        Color detectedColor = colorSensor.getColor();
        ColorMatchResult match = colorMatcher.matchClosestColor(detectedColor);

        if (match.color == blueBallColor) {
            colorString = "Blue";
        } else if (match.color == redBallColor) {
            colorString = "Red";
        } else {
            colorString = "Unknown";
        }

        return colorString;
    }


}
