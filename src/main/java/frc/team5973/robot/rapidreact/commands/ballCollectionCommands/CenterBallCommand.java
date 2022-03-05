package frc.team5973.robot.rapidreact.commands.ballCollectionCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team5973.robot.RobotBase;
import frc.team5973.robot.rapidreact.DetectionData;
import frc.team5973.robot.subsystems.SwerveDrive;

public class CenterBallCommand extends CommandBase {

	private final SwerveDrive drive;
    
    private DetectionData detectionData;

    private PIDController rotationSpeed;
    
    private boolean isFinished = false;

    private double imageWidth = detectionData.getResX();
    private double imageCenter = imageWidth / 2;


	public CenterBallCommand(RobotBase robot, SwerveDrive drive) {
		
		this.drive = drive;
	}

	@Override
	public void initialize() {

        rotationSpeed = new PIDController(0.04, 0, 0);
	}

	public void execute() {
        
        //TODO: need to determine the correct direction for the robot to rotate so it centers correctly
        if(detectionData.isBlueBallDetected()) {

            if(detectionData.getBlueBallCenterX() < imageCenter) {

                drive.rotate(rotationSpeed.calculate(detectionData.getBlueBallCenterX(), imageCenter), false);

            } else if(detectionData.getBlueBallCenterX() > imageCenter) {

                drive.rotate(-rotationSpeed.calculate(detectionData.getBlueBallCenterX(), imageCenter), false);

            } else {

                isFinished = true;

            }

        } else if (detectionData.isRedBallDetected()) {

            if(detectionData.getRedBallCenterX() < imageCenter) {

                drive.rotate(rotationSpeed.calculate(detectionData.getRedBallCenterX(), imageCenter), false);

            } else if(detectionData.getRedBallCenterX() > imageCenter) {

                drive.rotate(-rotationSpeed.calculate(detectionData.getRedBallCenterX(), imageCenter), false);

            } else {

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