package frc.team5973.robot.rapidreact.commands.ballCollectionCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team5973.robot.rapidreact.DetectionData;
import frc.team5973.robot.subsystems.SwerveDrive;

public class SearchForBallCommand extends CommandBase {

	private final SwerveDrive drive;
    
    private DetectionData detectionData;
    
    private boolean isFinished = false;

    private final double ROTATE_SPEED = 0.5;

	public SearchForBallCommand(SwerveDrive drive) {
		
		this.drive = drive;
	}

	@Override
	public void initialize() {
		

	}

	public void execute() {
        
        if(!detectionData.isBlueBallDetected() || !detectionData.isRedBallDetected()) {

            drive.rotate(ROTATE_SPEED, false);

        } else if (detectionData.isBlueBallDetected() || detectionData.isRedBallDetected()) {

            if(detectionData.isBlueBallDetected() && !detectionData.isRedBallDetected()) {

				System.out.println("blue ball detected");

				isFinished = true;

            } else if (!detectionData.isBlueBallDetected() && detectionData.isRedBallDetected()) {

				System.out.println("red ball detected");

				isFinished = true;
			}

        }

    
	
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