package frc.team5973.robot.rapidreact.commands.driveCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team5973.robot.subsystems.SwerveDrive;

public class StopDrive extends CommandBase {

	private final SwerveDrive drive;
	private boolean isFinished = false;

	public StopDrive(SwerveDrive drive) {

		this.drive = drive;

	}

	@Override
	public void initialize() {
		
		isFinished = false;

	}

	@Override
	public void execute() {

		// Set motor powers
		drive.halt();

		isFinished = true;
	
	}

	@Override
	public boolean isFinished() {

		return isFinished;
	
	}

	@Override
	public void end(boolean interrupted) {
		
	}
	
}
