package frc.team5973.robot.rapidreact.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team5973.robot.rapidreact.Shooter;
import frc.team5973.robot.subsystems.Limelight;
import frc.team5973.robot.subsystems.SwerveDrive;

public class AutoShootCommand extends CommandBase {

	private final SwerveDrive drive;
    private final Shooter shooter;
    
    private Limelight limelight;

    private PIDController pidForward;
    private PIDController pidStrafe;
    private PIDController pidRotate;

    private double forwardSpeedController;
    private double strafeSpeedController;
    private double rotateSpeedController;
    
    private final double targetDistance = 50; //TODO: need to figure out optimal distance

    private final double ROTATION_SPEED = 0.5;

    private boolean isFinished = false;


	public AutoShootCommand(SwerveDrive drive, Shooter shooter) {
		
		this.drive = drive;
        this.shooter = shooter;
	}

	@Override
	public void initialize() {

        pidForward = new PIDController(0.04, 0, 0);
        pidStrafe  = new PIDController(0.04, 0, 0);
        pidRotate  = new PIDController(0.04, 0, 0);

	}

	public void execute() {
        
        forwardSpeedController = MathUtil.clamp(pidForward.calculate(limelight.getDistance(), targetDistance), -1, 1);
        strafeSpeedController =  MathUtil.clamp(pidStrafe.calculate(limelight.getOffsetX(),   0             ), -1, 1);
        rotateSpeedController =  MathUtil.clamp(pidRotate.calculate(limelight.getOffsetX(),   0             ), -1, 1);

        if(!limelight.isTargetValid()) {
            
            drive.rotate(ROTATION_SPEED, false);

        } else {

            if(!(limelight.getDistance() <= targetDistance)) {

                drive.swerveDrive(forwardSpeedController, 
                                    strafeSpeedController, 
                                    rotateSpeedController,
                                    true);

            } else {

                drive.halt();
                shooter.shootForTime(5); //seconds //TODO: need to find time it takes for balls to get shot
            }
           
        }

        isFinished = true;
    
	}

	@Override
	public boolean isFinished() {

		return isFinished;
	
	}

	@Override
	public void end(boolean interrupted) {

        drive.halt();
	
	}

}