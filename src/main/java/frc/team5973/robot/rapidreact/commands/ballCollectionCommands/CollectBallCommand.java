package frc.team5973.robot.rapidreact.commands.ballCollectionCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team5973.robot.RobotBase;
import frc.team5973.robot.rapidreact.DetectionData;
import frc.team5973.robot.rapidreact.Intake;
import frc.team5973.robot.subsystems.SwerveDrive;

public class CollectBallCommand extends CommandBase {

	private final SwerveDrive drive;
    private final Intake intake;
    
    private DetectionData detectionData;
    
    private boolean isFinished = false;

    private final double targetDistance = 100;//TODO: need to find the correct distance to intake the balls

    private final double speed;

	public CollectBallCommand(RobotBase robot, SwerveDrive drive, Intake intake, double speed) {
		
		this.drive = drive;
        this.intake = intake;
        this.speed = speed;
	}

	@Override
	public void initialize() {

	}

	public void execute() {
        
        if(detectionData.isBlueBallDetected()) {
            
            while(detectionData.distanceFromTarget("blue") > targetDistance) {

                drive.driveForward(speed, false);
            
            }

            intake.intakeTime(1);

        } else if(detectionData.isRedBallDetected()) {
            
            while(detectionData.distanceFromTarget("red") > targetDistance) {

                drive.driveForward(speed, false);
            
            }

            intake.intakeTime(1);

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
        intake.halt();
	
	}

}