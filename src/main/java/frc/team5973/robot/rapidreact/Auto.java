package frc.team5973.robot.rapidreact;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.team5973.robot.rapidreact.commands.AutoShootCommand;

public class Auto extends SequentialCommandGroup {

	final double speed = 0.7;
	
	public Auto() {

		addCommands(

			//new AutoShootCommand()

			
		);


	}

}