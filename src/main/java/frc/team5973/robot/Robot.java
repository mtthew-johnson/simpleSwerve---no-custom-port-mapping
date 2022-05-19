package frc.team5973.robot;

import java.util.Map;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.team5973.robot.commands.SwerveDriveDefaultCommand;
import frc.team5973.robot.subsystems.SwerveDrive;
import frc.team5973.robot.subsystems.SwerveDrive.Axis;
import frc.team5973.robot.subsystems.SwerveDrive.DriveMode;

public class Robot extends TimedRobot {
	
	private SwerveDrive drive;

	private Command autonomousCommand;

	private SendableChooser<Command> chooser;

	private int driverPort  = 0;
	private int copilotPort = 1;

	private final double SPEED_NORMAL  = 0.9;
	private final double SPEED_SAFE    = 0.3;
	
	private final double DEADBAND  = 0.07;

	private XboxController driver  = new XboxController(driverPort);
	private XboxController copilot = new XboxController(copilotPort);

	public Robot() {

		drive = new SwerveDrive();

		drive.setDefaultCommand(new SwerveDriveDefaultCommand(drive, 
															  DEADBAND,
															  SPEED_NORMAL, 
															  SPEED_SAFE, 
															  Map.of(
			Axis.FORWARD, () -> driver.getLeftY(),
			Axis.STRAFE,  () -> driver.getLeftX(),
			Axis.ROTATE,    () -> driver.getRightX()),
			Map.of(
			DriveMode.SAFEMMODE, () -> driver.getLeftStickButton() && driver.getRightStickButton(),
			DriveMode.FIELDMODE, () -> copilot.getBButton(),
			DriveMode.ZERO_GYRO, () -> copilot.getXButton()
			)));

	}

	@Override
	public void robotInit() {

	}

	@Override
	public void teleopInit() {
			
		if (autonomousCommand != null)
			autonomousCommand.cancel();
	}

	@Override
	public void teleopPeriodic() {
		CommandScheduler.getInstance().run();
	}

	@Override
	public void testPeriodic() {
		CommandScheduler.getInstance().run();
		
	}

	@Override
	public void disabledInit() {
		if (autonomousCommand != null) {
			autonomousCommand.cancel();
			autonomousCommand = null;
		}
	}

	@Override
	public void autonomousInit() {

		autonomousCommand = chooser.getSelected();

		// schedule the autonomous command (example)
		if (autonomousCommand != null) {
			autonomousCommand.schedule();
		  }
		
	}

	@Override
	public void autonomousPeriodic() {
		CommandScheduler.getInstance().run();
	}

	@Override
	public void startCompetition() {
		// TODO Auto-generated method stub
		
	}

	@Override
	public void endCompetition() {
		// TODO Auto-generated method stub
		
	}

}
