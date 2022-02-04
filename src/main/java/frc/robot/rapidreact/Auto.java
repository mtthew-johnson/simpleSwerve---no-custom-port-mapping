package frc.robot.rapidreact;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.rapidreact.commands.driveCommands.StopDrive;
import frc.robot.rapidreact.commands.driveCommands.driveForTimeCommands.DriveForTimeForward;
import frc.robot.subsystems.SwerveDrive;


public class Auto extends SequentialCommandGroup {
	
	public Auto(SwerveDrive drive) {

		addCommands(

        	new DriveForTimeForward(1, 0.5, drive),
			new StopDrive(drive)
			
		);


	}

}