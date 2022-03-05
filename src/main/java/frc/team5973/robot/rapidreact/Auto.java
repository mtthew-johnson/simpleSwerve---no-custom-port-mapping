package frc.team5973.robot.rapidreact;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.team5973.robot.RobotBase;
import frc.team5973.robot.rapidreact.commands.AutoShootCommand;
import frc.team5973.robot.rapidreact.commands.ballCollectionCommands.CenterBallCommand;
import frc.team5973.robot.rapidreact.commands.ballCollectionCommands.CollectBallCommand;
import frc.team5973.robot.rapidreact.commands.ballCollectionCommands.SearchForBallCommand;
import frc.team5973.robot.rapidreact.commands.driveCommands.StopDrive;
import frc.team5973.robot.subsystems.SwerveDrive;


public class Auto extends SequentialCommandGroup {

	final double speed = 0.7;
	
	public Auto(RobotBase robot, SwerveDrive drive, Intake intake, Shooter shooter) {

		addCommands(

			new StopDrive(drive),

			new SearchForBallCommand(drive),
			new CenterBallCommand(robot, drive),
			new CollectBallCommand(robot, drive, intake, speed),
			new AutoShootCommand(drive, shooter)
			
		);


	}

}