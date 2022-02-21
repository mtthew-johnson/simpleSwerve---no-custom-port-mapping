package frc.robot.rapidreact.shooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.motorcontrol.Jaguar;
import frc.robot.RobotBase;
import frc.robot.subsystems.SubsystemBase;

public class Shooter extends SubsystemBase {
    
    private final double SHOOTER_SPEED = 1;
    private final double OUTAKE_SPEED = 0.3;

    private Jaguar shooterWheel;
    private Jaguar shooterOutake;

    public enum ShooterInput {BUTTON}

    private final Timer timer = new Timer();
    
    public Shooter(RobotBase robot) {

        super(robot);

        configureMotors();

    }

    private void configureMotors() {
        
        shooterWheel = new Jaguar(1);//new Jaguar(configInt("shooterWheel"));
        shooterOutake = new Jaguar(0);//new Jaguar(configInt("shooterOutake"));

        addChild("shooterWheel", shooterWheel);
        addChild("shooterOutake", shooterOutake);

        shooterWheel.setInverted(false);
        shooterOutake.setInverted(true);
    }

    public void shoot() {
            
            //spin up flywheel
            shooterWheel.set(SHOOTER_SPEED);
            
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

}
