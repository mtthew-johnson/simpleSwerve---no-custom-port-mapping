package frc.team5973.robot.commands;

import java.util.Map;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.team5973.robot.subsystems.SwerveDrive;
import frc.team5973.robot.subsystems.SwerveDrive.Axis;
import frc.team5973.robot.subsystems.SwerveDrive.DriveMode;

public class SwerveDriveDefaultCommand extends CommandBase {
    final SwerveDrive drive;
    final Map<Axis, DoubleSupplier> axisMap;
    final Map<DriveMode, BooleanSupplier> buttonMap;

    private final double DEADBAND;

    private final double SPEED_NORMAL;
    private final double SPEED_SAFE;

    private double speed;
 
    private double comboStartTimeSafe  = 0;
    private double comboStartTimeField = 0;
    
    private boolean alreadyToggledSafeMode  = false;
    private boolean alreadyToggledFieldMode = false;
 
    private boolean safeMode          = false;
    private boolean fieldOrientedMode = false;

    private double buttonDelay = 0.0; //increase this value in seconds to add a button delay for switching drive modes

    private double forward;
    private double strafe;
    private double rotate;

    public double yawCorrection;
   
    private int driveMode = 1;

    private final int fieldOriented = 1;
    private final int robotOriented = 2;
 
    public SwerveDriveDefaultCommand(final SwerveDrive drive,
                                     final double DEADBAND,
                                     final double SPEED_NORMAL,
                                     final double SPEED_SAFE,
                                     final Map<Axis, DoubleSupplier> axisMap,
                                     final Map<DriveMode, BooleanSupplier> buttonMap) {
        

        this.buttonMap     = buttonMap;
        this.axisMap       = axisMap;
        this.drive         = drive;
        
        this.DEADBAND     = DEADBAND;
        this.SPEED_NORMAL = SPEED_NORMAL;
        this.SPEED_SAFE   = SPEED_SAFE;
    
        addRequirements(drive);
        SendableRegistry.addChild(SwerveDriveDefaultCommand.this, this);

    }

    @Override
	public void execute() {
        
        //determine whether to use safe mode or not
        speed = safeMode ? SPEED_SAFE : SPEED_NORMAL;
        
        //controller inputs
        forward =   MathUtil.clamp(MathUtil.applyDeadband(axis(Axis.FORWARD), DEADBAND) * speed, -1, 1);
        strafe  =  -MathUtil.clamp(MathUtil.applyDeadband(axis(Axis.STRAFE),  DEADBAND) * speed, -1, 1);
        rotate  =  -MathUtil.clamp(MathUtil.applyDeadband(axis(Axis.ROTATE),       0.1) * speed, -1, 1);

        //yaw correction calculation - think of this as a gyro straight function for you FLL people
        yawCorrection = drive.correctHeading(0.004, forward, strafe, rotate);
        

        //puts robot into safemode where the robot will go slower - uses button toggle
         if(button(DriveMode.SAFEMMODE)) {

            if(comboStartTimeSafe == 0)
                comboStartTimeSafe = Timer.getFPGATimestamp();
            else if(Timer.getFPGATimestamp() - comboStartTimeSafe >= 3.0 && !alreadyToggledSafeMode) {
            
                safeMode = !safeMode;
                
                alreadyToggledSafeMode = true;
                System.out.println("Safemode is " + (safeMode ? "Enabled" : "Disabled") + ".");
            
            }

        } else {

            comboStartTimeSafe = 0;
            alreadyToggledSafeMode = false;
        
        }

        // toggle POV and field mode
        if(button(DriveMode.FIELDMODE)) {

            if(comboStartTimeField == 0) {
                comboStartTimeField = Timer.getFPGATimestamp();
            } else if(Timer.getFPGATimestamp() - comboStartTimeField >= buttonDelay && !alreadyToggledFieldMode) {
                
                fieldOrientedMode = !fieldOrientedMode;

                alreadyToggledFieldMode = true;
                driveMode = fieldOrientedMode ? fieldOriented : robotOriented;
                System.out.println("Switching to " + (fieldOrientedMode ? "Field Oriented" : "Robot POV") + ".");
            }

        } else {

            comboStartTimeField = 0;
            alreadyToggledFieldMode = false;
            
        }

        if(button(DriveMode.ZERO_GYRO)) {
            drive.resetGyro();

            yawCorrection = 0;

            System.out.println("Gyro reset");
        }

        //set drive modes
        switch (driveMode) {
            case fieldOriented: drive.swerveDrive(forward, strafe, rotate - yawCorrection, true); //you might have to add or subtract the yawCorrection - depending on your gyro setup
                    break;
            case robotOriented: drive.swerveDrive(forward, strafe, rotate - yawCorrection, false);
                    break;
            default: drive.swerveDrive(forward, strafe, rotate - yawCorrection, true);
                     break;
        }
	}

	private final double axis(Axis axis) {
		return axisMap.get(axis).getAsDouble();
	}

    private final boolean button(DriveMode button) {
        return buttonMap.get(button).getAsBoolean();
    }

	@Override
	public void initSendable(SendableBuilder builder) {
		super.initSendable(builder);

        builder.addDoubleProperty("hello", () -> yawCorrection, null);

        
		
	}

}
