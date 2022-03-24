package frc.team5973.robot.rapidreact.commands;

import java.util.Map;
import java.util.function.DoubleSupplier;

import org.ejml.dense.block.decomposition.chol.InnerCholesky_DDRB;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team5973.robot.rapidreact.Intake;
import frc.team5973.robot.rapidreact.Shooter;

import frc.team5973.robot.subsystems.Limelight;
import frc.team5973.robot.subsystems.SwerveDrive;
import frc.team5973.robot.subsystems.SwerveDrive.Axis;

public class DriveDistance extends CommandBase {

    private final SwerveDrive drive;
	
    private boolean isFinished = false;
    
    private double distance;
    
    public DriveDistance(double inches, final SwerveDrive drive) {
        
        //addRequirements(drive);
        distance = inches;

    this.drive = drive;
	}

	@Override
	public void initialize() {
        drive.resetDriveSensors();

	}

	public void execute() {
        drive.swerveDrive(-0.3, 0, 0, false);
	}

	@Override
	public boolean isFinished() {

        System.out.println("Done");

        return drive.getAverageEncoderPosition() >= (40960 / (12 * Math.PI)) * (distance);
	
	}

	@Override
	public void end(boolean interrupted) {

        
        //drive.halt();
        System.out.println("done!!");
	
	}


}