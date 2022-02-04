package frc.robot.rapidreact;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotBase;
import frc.robot.subsystems.SubsystemBase;

public class Shooter extends SubsystemBase {
    
    private final double SHOOTER_SPEED = 1;
    private final double OUTAKE_SPEED = 0.5;

    private WPI_TalonSRX shooterWheel;
    private WPI_TalonSRX shooterOutake;

    private final Timer timer = new Timer();
    
    public Shooter(RobotBase robot) {

        super(robot);

        initMotors();

        initDefaultCommand();

    }

    private void initMotors() {
        
        shooterWheel = new WPI_TalonSRX(configInt("shooterWheel"));
        shooterOutake = new WPI_TalonSRX(configInt("shooterOutake"));

        shooterWheel.setInverted(false);
        shooterOutake.setInverted(false);
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
