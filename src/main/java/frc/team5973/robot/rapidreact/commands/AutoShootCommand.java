package frc.team5973.robot.rapidreact.commands;

import java.util.Map;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team5973.robot.rapidreact.Intake;
import frc.team5973.robot.rapidreact.Shooter;

import frc.team5973.robot.subsystems.Limelight;
import frc.team5973.robot.subsystems.SwerveDrive;
import frc.team5973.robot.subsystems.SwerveDrive.Axis;

public class AutoShootCommand extends CommandBase {
    
    private final SwerveDrive drive;
    private final Shooter shooter;
    private final Intake intake;
    
    private final Limelight limelight;
    private boolean isFinished = false;

    private double shooterSpeed = 0;

    private Timer timer = new Timer();

	public AutoShootCommand(final SwerveDrive drive, final Shooter shooter, final Intake intake, final Limelight limelight) {

        this.drive = drive;
        this.shooter = shooter;
        this.intake = intake;
        this.limelight = limelight;

	}

	@Override
	public void initialize() {

        


	}

	public void execute() {

        //yawCorrection = drive.correctHeading(0.004, -0.3, 0, 0);

        // drive.driveDistance(77, -0.3, 0, 0);
        // drive.halt();
        // intake.extend();
        // intake.collect();

        // timer.reset();
        // timer.start();
        // while(timer.hasElapsed(2)){}
       
        // intake.retract();

        // timer.reset();
        // timer.start();
        // while(timer.hasElapsed(2)){}

        // intake.halt();


        //     drive.swerveDrive(0, 0, -limelight.limelightXPID(), false);
        // delay(1);

        // if(limelight.getOffsetY() <= 13.5) {
        //     shooterSpeed = 0.9670 - (0.03439       * limelight.getOffsetY())
        //                           + (0.002301      * Math.pow(limelight.getOffsetY(), 2)) 
        //                           + (0.0001523     * Math.pow(limelight.getOffsetY(), 3))
        //                           - (0.0000181     * Math.pow(limelight.getOffsetY(), 4))
        //                           + (0.00000009149 * Math.pow(limelight.getOffsetY(), 5));
        // } else if(limelight.getOffsetY() > 13.5) {
        //     shooterSpeed = 4.510 - (0.9352       * limelight.getOffsetY()) 
        //                          + (0.09102      * Math.pow(limelight.getOffsetY(), 2)) 
        //                          - (0.004322     * Math.pow(limelight.getOffsetY(), 3))
        //                          + (0.00009939   * Math.pow(limelight.getOffsetY(), 4))
        //                          - (0.0000008843 * Math.pow(limelight.getOffsetY(), 5));
        // }

        // shooter.shoot(shooterSpeed);
        // delay(1);
        // intake.collect();
        // delay(1);

    //     shooter.halt();
    //     intake.halt();

    //////////////////////////////////////////////////////////////

    // intake.extend();
    // intake.collect();
    // drive.resetGyro();
    //drive.driveDistance(74, -0.2, 0, 0);
    //drive.halt();

   // drive.swerveDrive(-0.3, 0, 0, false);
    
    // intake.collect();
    // intake.extend();

    // timer.reset();
    // timer.start();
    // while(!timer.hasElapsed(2)) {}
    // drive.halt();

    // intake.retract();

    // timer.reset();
    // timer.start();
    // while(!timer.hasElapsed(1)) {}

    // intake.halt();

//     drive.swerveDrive(0, 0, -limelight.limelightXPID(), false);
//     timer.reset();
//     timer.start();
//     while(!timer.hasElapsed(1)) {}

//     drive.halt();

//      if(limelight.getOffsetY() <= 13.5) {
//             shooterSpeed = 0.9670 - (0.03439       * limelight.getOffsetY())
//                                   + (0.002301      * Math.pow(limelight.getOffsetY(), 2)) 
//                                   + (0.0001523     * Math.pow(limelight.getOffsetY(), 3))
//                                   - (0.0000181     * Math.pow(limelight.getOffsetY(), 4))
//                                   + (0.00000009149 * Math.pow(limelight.getOffsetY(), 5));
//         } else if(limelight.getOffsetY() > 13.5) {
//             shooterSpeed = 4.510 - (0.9352       * limelight.getOffsetY()) 
//                                  + (0.09102      * Math.pow(limelight.getOffsetY(), 2)) 
//                                  - (0.004322     * Math.pow(limelight.getOffsetY(), 3))
//                                  + (0.00009939   * Math.pow(limelight.getOffsetY(), 4))
//                                  - (0.0000008843 * Math.pow(limelight.getOffsetY(), 5));
//         }
    
//     shooter.shoot(shooterSpeed);

//     timer.reset();
//     timer.start();
//     while(!timer.hasElapsed(2)) {}

//     intake.collect();

//     timer.reset();
//     timer.start();
//     while(!timer.hasElapsed(2)) {}

//    shooter.halt();

//    timer.reset();
//    timer.start();
//    while(!timer.hasElapsed(1)) {}

//    drive.rotateDegrees(70);



//    intake.extend();
//    drive.driveDistance(75, -0.2, 0, 0);
//    drive.halt();
    drive.resetDriveSensors();
    drive.driveDistance(25, -0.2, 0, 0);
    //drive.swerveDrive(-0.2, 0, 0, false);

    // timer.reset();
    // timer.start();
    // while(!timer.hasElapsed(2)){}
    // timer.stop();

    // drive.swerveDrive(0, 0, 0 - limelight.limelightXPID(), false);
    // timer.reset();
    // timer.start();
    // while(!timer.hasElapsed(1)) {}
    // timer.stop();


    //      if(limelight.getOffsetY() <= 13.5) {
    //         shooterSpeed = 0.9670 - (0.03439       * limelight.getOffsetY())
    //                               + (0.002301      * Math.pow(limelight.getOffsetY(), 2)) 
    //                               + (0.0001523     * Math.pow(limelight.getOffsetY(), 3))
    //                               - (0.0000181     * Math.pow(limelight.getOffsetY(), 4))
    //                               + (0.00000009149 * Math.pow(limelight.getOffsetY(), 5));
    //     } else if(limelight.getOffsetY() > 13.5) {
    //         shooterSpeed = 4.510 - (0.9352       * limelight.getOffsetY()) 
    //                              + (0.09102      * Math.pow(limelight.getOffsetY(), 2)) 
    //                              - (0.004322     * Math.pow(limelight.getOffsetY(), 3))
    //                              + (0.00009939   * Math.pow(limelight.getOffsetY(), 4))
    //                              - (0.0000008843 * Math.pow(limelight.getOffsetY(), 5));
    //     }
    
    // shooter.shoot(shooterSpeed);

    // timer.reset();
    // timer.start();
    // while(!timer.hasElapsed(2)) {}
    // timer.stop();

    // intake.extend();
    // intake.collect();

    // timer.reset();
    // timer.start();
    // while(!timer.hasElapsed(3)) {}

    System.out.println("Executed");


    isFinished = true;

	}

	@Override
	public boolean isFinished() {

        System.out.println("Done");

        return isFinished;
	
	}

    public void delay(double delayTime) {
        timer.reset();
        timer.start();
        
        while(timer.hasElapsed(delayTime)) {}
    }

	@Override
	public void end(boolean interrupted) {

        
        //drive.halt();
        System.out.println("done!!");
	
	}


}