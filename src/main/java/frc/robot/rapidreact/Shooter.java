package frc.robot.rapidreact;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.motorcontrol.Jaguar;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotBase;
import frc.robot.subsystems.SubsystemBase;

public class Shooter extends SubsystemBase {
    
    private final double SHOOTER_SPEED = 1;
    private final double OUTAKE_SPEED = 0.3;

    private Jaguar shooterWheel;
    private Jaguar shooterOutake;

    private final Timer timer = new Timer();
    
    public Shooter(RobotBase robot) {

        super(robot);

        initMotors();

        initDefaultCommand();

    }

    private void initMotors() {
        
        shooterWheel = new Jaguar(1);
        shooterOutake = new Jaguar(0);

        shooterWheel.setInverted(false);
        shooterOutake.setInverted(true);
    }

    private void shoot() {
        
        if(button("shoot")) {
                    
            //spin up flywheel
            shooterWheel.set(SHOOTER_SPEED);
            
            // Wait for 1 second to allow wheel to spin up   
            timer.reset();
            timer.start();
            while (!timer.hasElapsed(1)) {}
            timer.stop();

            //send ball through
            shooterOutake.set(OUTAKE_SPEED);

        } else {

            shooterWheel.set(0);
            shooterOutake.set(0);
        }
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

    public void initDefaultCommand() {

		setDefaultCommand(new CommandBase() {

			{
				addRequirements(Shooter.this);
			}

			@Override
			public void execute() {
				
                shoot();

			}

		});

	}

}
