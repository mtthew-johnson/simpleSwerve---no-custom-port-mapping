package frc.robot.subsystems;

import java.util.Map;
import java.util.function.DoubleSupplier;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.SwerveBase.Input;

public class SwerveDriveDefaultCommand extends CommandBase {
    final SwerveDrive drive;
    final Gyro gyro;
    final Map<Input, DoubleSupplier> inputMap;

    
    private boolean safeMode = false;
    private double comboStartTime = 0;
    private boolean alreadyToggled = false;

    private boolean fieldOrientedMode = false;

   
    public SwerveDriveDefaultCommand(final SwerveDrive drive, final Gyro gyro, Map<Input, DoubleSupplier> inputMap) {
        this.drive = drive;
        this.gyro = gyro;
        this.inputMap = inputMap;

        addRequirements(drive);
        SendableRegistry.addChild(SwerveDriveDefaultCommand.this, this);

    }

    @Override
	public void execute() {
         //puts robot into safemode where the robot will go slower
         if (drive.getSafeMode()) {

            if (comboStartTime == 0)
                comboStartTime = Timer.getFPGATimestamp();
            else if (Timer.getFPGATimestamp() - comboStartTime >= 3.0 && !alreadyToggled) {
            
                safeMode = !safeMode;
                drive.setSafeMode(safeMode);
                alreadyToggled = true;
                System.out.println("Safemode is " + (safeMode ? "Enabled" : "Disabled") + ".");
            
            }

        } else {

            comboStartTime = 0;
            alreadyToggled = false;
        
        }

        // toggle POV and field mode
        if (drive.getIsFieldOriented()) {

            fieldOrientedMode = !fieldOrientedMode;
            drive.setIsFieldOriented(fieldOrientedMode);
            gyro.reset();

        }
 
        drive.swerveDrive(axis(Input.FORWARD), 
                          axis(Input.STRAFE), 
                          axis(Input.TURN), 
                          fieldOrientedMode);
          
        drive.calculateRobotPosition();
	
	}

	private final double axis(Input input) {
		return inputMap.get(input).getAsDouble();
	}

	@Override
	public void initSendable(SendableBuilder builder) {
		super.initSendable(builder);
		
	}

}
