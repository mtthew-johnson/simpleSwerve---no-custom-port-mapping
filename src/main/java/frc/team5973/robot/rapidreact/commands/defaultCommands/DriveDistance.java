package frc.team5973.robot.rapidreact.commands.defaultCommands;

import java.util.Map;
import java.util.function.BooleanSupplier;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team5973.robot.rapidreact.Intake;
import frc.team5973.robot.rapidreact.Intake.IntakeInput;
import frc.team5973.robot.subsystems.SwerveDrive;

public class DriveDistance extends CommandBase {
    final Intake intake;
    final SwerveDrive drive;

    boolean isFinished = false;

    public DriveDistance(final Intake intake, final SwerveDrive drive) {
        this.intake = intake;
        this.drive = drive;

        addRequirements(intake, drive);
        SendableRegistry.addChild(DriveDistance.this, this);

    }

    @Override
	public void execute() {

        System.out.println("executed");

        drive.resetDriveSensors();
        drive.driveDistance(12, -0.3, 0, 0);

        isFinished = true;
        
       



	
	}

    @Override
    public boolean isFinished() {
        System.out.println("Done");
        return isFinished;
    }

	@Override
	public void initSendable(SendableBuilder builder) {
		super.initSendable(builder);
		
	}


}
