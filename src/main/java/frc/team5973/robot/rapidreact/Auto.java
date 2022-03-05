package frc.team5973.robot.rapidreact;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.team5973.robot.RobotBase;
import frc.team5973.robot.rapidreact.commands.AutoShoot;
import frc.team5973.robot.rapidreact.commands.ballCollectionCommands.CenterBall;
import frc.team5973.robot.rapidreact.commands.ballCollectionCommands.CollectBall;
import frc.team5973.robot.rapidreact.commands.ballCollectionCommands.SearchForBall;
import frc.team5973.robot.rapidreact.commands.driveCommands.StopDrive;
import frc.team5973.robot.subsystems.SwerveDrive;


public class Auto extends SequentialCommandGroup {

	final double speed = 0.7;
	
	public Auto(RobotBase robot, SwerveDrive drive, Intake intake, Shooter shooter) {

		addCommands(

			new StopDrive(drive),

			new SearchForBall(drive),
			new CenterBall(robot, drive),
			new CollectBall(robot, drive, intake, speed),
			new AutoShoot(drive, shooter)
			
		);


	}

}