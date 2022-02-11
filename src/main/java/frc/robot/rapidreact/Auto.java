package frc.robot.rapidreact;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotBase;
import frc.robot.rapidreact.commands.AutoShoot;
import frc.robot.rapidreact.commands.ballCollectionCommands.CenterBall;
import frc.robot.rapidreact.commands.ballCollectionCommands.CollectBall;
import frc.robot.rapidreact.commands.ballCollectionCommands.SearchForBall;
import frc.robot.rapidreact.commands.driveCommands.StopDrive;
import frc.robot.rapidreact.commands.driveCommands.driveForTimeCommands.DriveForTimeForward;
import frc.robot.subsystems.SwerveDrive;


public class Auto extends SequentialCommandGroup {

	final double speed = 0.7;
	
	public Auto(RobotBase robot, SwerveDrive drive, Intake intake, Shooter shooter) {

		addCommands(

        	new DriveForTimeForward(1, 0.5, drive),
			new StopDrive(drive),

			new SearchForBall(drive),
			new CenterBall(robot, drive),
			new CollectBall(robot, drive, intake, speed),
			new AutoShoot(drive, shooter)
			
		);


	}

}