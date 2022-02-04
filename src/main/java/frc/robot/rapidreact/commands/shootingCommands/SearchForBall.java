package frc.robot.rapidreact.commands.shootingCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.rapidreact.DetectionData;
import frc.robot.subsystems.SwerveDrive;

public class SearchForBall extends CommandBase {

	private final SwerveDrive drive;
    
    private DetectionData detectionData;
    
    private boolean isFinished = false;

    private final double ROTATE_SPEED = 0.5;

	public SearchForBall(SwerveDrive drive) {
		
		this.drive = drive;
	}

	@Override
	public void initialize() {
		

	}

	public void execute() {

        isFinished = true;
        
        if(!detectionData.isBlueBallDetected() || !detectionData.isRedBallDetected()) {

            drive.rotate(ROTATE_SPEED);

        } else {

            drive.halt();

            if(detectionData.isBlueBallDetected() && !detectionData.isRedBallDetected()) {

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