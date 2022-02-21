package frc.robot.subsystems;

import java.util.Map;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.rapidreact.DetectionData;
import frc.robot.subsystems.SwerveDrive.Axis;
import frc.robot.subsystems.SwerveDrive.DriveMode;

public class SwerveDriveDefaultCommand extends CommandBase {
    final SwerveDrive drive;
    final Limelight limelight;
    final DetectionData detectionData;
    final Gyro gyro;
    final Map<Axis, DoubleSupplier> axisMap;
    final Map<DriveMode, BooleanSupplier> buttonMap;

    private final double DEADBAND;

    private final double SPEED_NORMAL;
    private final double SPEED_SAFE;

    private double speed;

    private boolean safeMode = false;
    private double comboStartTime = 0;
    private boolean alreadyToggledSafeMode = false;

    private boolean fieldOrientedMode = false;
    private boolean goalOrientedMode  = false;
    private boolean ballOrientedMode  = false;

    private double forward;
    private double strafe;
    private double rotate;

    private double yawCorrection;
   
    private int driveMode;

    private final int fieldOriented = 1;
    private final int robotOriented = 2;
    private final int goalOriented  = 3;
    private final int ballOriented  = 4;

    public SwerveDriveDefaultCommand(final SwerveDrive drive,
                                     final Limelight limelight,
                                     final DetectionData detectionData, 
                                     final Gyro gyro,
                                     final double SPEED_NORMAL,
                                     final double SPEED_SAFE,
                                     final double DEADBAND, 
                                     final Map<Axis, DoubleSupplier> axisMap,
                                     final Map<DriveMode, BooleanSupplier> buttonMap) {
        
        this.detectionData = detectionData;
        this.limelight     = limelight;
        this.buttonMap     = buttonMap;
        this.axisMap       = axisMap;
        this.drive         = drive;
        this.gyro          = gyro;
        
        this.SPEED_NORMAL = SPEED_NORMAL;
        this.SPEED_SAFE   = SPEED_SAFE;
        this.DEADBAND     = DEADBAND;

        addRequirements(drive);
        SendableRegistry.addChild(SwerveDriveDefaultCommand.this, this);

    }

    @Override
	public void execute() {
        
        speed = safeMode ? SPEED_SAFE : SPEED_NORMAL;
        
        forward =  MathUtil.applyDeadband(axis(Axis.FORWARD), DEADBAND) * speed;
        strafe  = -MathUtil.applyDeadband(axis(Axis.STRAFE),  DEADBAND) * speed;
        rotate  = -MathUtil.applyDeadband(axis(Axis.TURN),    DEADBAND) * speed;

        yawCorrection = drive.correctHeading(gyro.getAngle(), 0.004, forward, strafe, rotate);
        
        //puts robot into safemode where the robot will go slower
         if (button(DriveMode.SAFEMMODE)) {

            if (comboStartTime == 0)
                comboStartTime = Timer.getFPGATimestamp();
            else if (Timer.getFPGATimestamp() - comboStartTime >= 3.0 && !alreadyToggledSafeMode) {
            
                safeMode = !safeMode;
                alreadyToggledSafeMode = true;
                System.out.println("Safemode is " + (safeMode ? "Enabled" : "Disabled") + ".");
            
            }

        } else {

            comboStartTime = 0;
            alreadyToggledSafeMode = false;
        
        }

         // toggle POV and field mode
         if (button(DriveMode.FIELDMODE) && !button(DriveMode.GOALMODE) && !button(DriveMode.BALLMODE)) {

            fieldOrientedMode = !fieldOrientedMode;

            driveMode = fieldOrientedMode ? fieldOriented : robotOriented;
            System.out.println("Switching to " + (fieldOrientedMode ? "Field Oriented" : "Robot POV") + ".");

        }

        //toggle goal centric mode
        if(button(DriveMode.GOALMODE) && !button(DriveMode.FIELDMODE) && !button(DriveMode.BALLMODE) && limelight.isTargetValid()) {
            
            goalOrientedMode = !goalOrientedMode;

            driveMode = goalOrientedMode ? goalOriented : fieldOriented;
            System.out.println("Switching to " + (goalOrientedMode ? "Goal Oriented" : "Field Oriented") + ".");

        } else if (button(DriveMode.GOALMODE) && !button(DriveMode.FIELDMODE) && !limelight.isTargetValid()) {

            driveMode = fieldOriented;
            System.out.println("No valid target to change drive mode" + "\n Switching to Field Oriented Mode");
        }

        //toggle ball centric mode
        if(button(DriveMode.BALLMODE) && !button(DriveMode.GOALMODE) && !button(DriveMode.FIELDMODE) && detectionData.isAnyBallDetected()) {

            ballOrientedMode = !goalOrientedMode;

            driveMode = ballOrientedMode ? ballOriented : fieldOriented;
            System.out.println("Switching to " + (ballOrientedMode ? "Ball Oriented" : "Field Oriented") + ".");

        } else if(button(DriveMode.BALLMODE) && !button(DriveMode.GOALMODE) && !button(DriveMode.FIELDMODE) && !detectionData.isAnyBallDetected()) {

            driveMode = fieldOriented;
            System.out.println("No valid target to change drive mode" + "\n Switching to Field Oriented Mode");
        }

        //set drive mode
        switch (driveMode) {
            case fieldOriented: drive.calculateDrive(forward, strafe, rotate + yawCorrection, gyro.getAngle(), true);
                    break;
            case robotOriented: drive.calculateDrive(forward, strafe, rotate + yawCorrection, gyro.getAngle(), false);
                    break;
            case goalOriented: drive.calculateDrive(forward + limelight.limelightYPID(), 
                                                    strafe, 
                                                    rotate + limelight.limelightXPID() + yawCorrection, 
                                                    gyro.getAngle(), 
                                                    false);
                    break;
            case ballOriented: drive.calculateDrive(-forward + detectionData.piYPID(), 
                                                    -strafe, 
                                                    -rotate + detectionData.piXPID() + yawCorrection, 
                                                    gyro.getAngle(), 
                                                    false);
                    break;
            default: drive.calculateDrive(forward, strafe, rotate + yawCorrection, gyro.getAngle(), true);
                     break;
        }
          
        //robot odometry
        drive.calculateRobotPosition();
	
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

        
		
	}

}
