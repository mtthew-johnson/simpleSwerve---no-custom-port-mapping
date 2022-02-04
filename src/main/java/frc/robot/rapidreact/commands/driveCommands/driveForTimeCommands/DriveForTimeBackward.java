package frc.robot.rapidreact.commands.driveCommands.driveForTimeCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.SwerveDrive;

public class DriveForTimeBackward extends CommandBase {
	
	private final Timer timer = new Timer();
	private final double speed;
	private final double time; //time is in seconds
	private final SwerveDrive drive;

	public DriveForTimeBackward(double time, double speed, SwerveDrive drive) {
		
		this.time = time;
		this.speed = speed;
		this.drive = drive;
	
	}

	@Override
	public void initialize() {
		
		timer.reset();
		timer.start();

	}

	public void execute() {

        drive.driveBackward(speed);
	
	}

	@Override
	public boolean isFinished() {

		return timer.hasElapsed(time);
	
	}

	@Override
	public void end(boolean interrupted) {

		timer.stop();

        drive.halt();
	
	}

}