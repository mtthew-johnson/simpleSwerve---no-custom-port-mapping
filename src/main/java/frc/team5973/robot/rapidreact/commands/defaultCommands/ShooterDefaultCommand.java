package frc.team5973.robot.rapidreact.commands.defaultCommands;

import java.util.Map;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team5973.robot.rapidreact.Shooter;
import frc.team5973.robot.rapidreact.Shooter.ShooterInput;
import frc.team5973.robot.rapidreact.Shooter.ShooterAxis;
import frc.team5973.robot.subsystems.Limelight;

public class ShooterDefaultCommand extends CommandBase {
    final Shooter shooter;
    final Limelight limelight;
    final Map<ShooterInput, BooleanSupplier> inputMap;
    final Map<ShooterAxis, DoubleSupplier> axisMap;

    private double shooterSpeed = 0;
    private double internalControl = 0;

    public ShooterDefaultCommand(final Shooter shooter, final Limelight limelight, Map<ShooterInput, BooleanSupplier> inputMap,
                                                                                   Map<ShooterAxis, DoubleSupplier> axisMap) {
        this.limelight = limelight;
        this.shooter = shooter;
        this.inputMap = inputMap;
        this.axisMap = axisMap;

        addRequirements(shooter);
        SendableRegistry.addChild(ShooterDefaultCommand.this, this);
    }

    @Override
	public void execute() {

        internalControl = -MathUtil.applyDeadband(axis(ShooterAxis.INTERNAL_WHEEL), 0.1) * 0.3;
        
        if(limelight.getOffsetY() <= 13.5) {
            shooterSpeed = 0.9730 - (0.04443 * limelight.getOffsetY()) +
                           ((0.008168) * Math.pow(limelight.getOffsetY(), 2)) - (0.0009408 * Math.pow(limelight.getOffsetY(), 3))
                                                                              +  (0.000066 * Math.pow(limelight.getOffsetY(), 4))
                                                                              -  (0.000002215 * Math.pow(limelight.getOffsetY(), 5));
        } else if(limelight.getOffsetY() > 13.5) {
            shooterSpeed = 4.533 - (0.9352 * limelight.getOffsetY()) + (0.09102 * Math.pow(limelight.getOffsetY(), 2)) - 
                                                                        (0.004322 * Math.pow(limelight.getOffsetY(), 3))
                                                                         + (0.00009939 * Math.pow(limelight.getOffsetY(), 4))
                                                                         - (0.0000008843 * Math.pow(limelight.getOffsetY(), 5));
        }

        System.out.println(shooterSpeed);
        
       

        if(Math.abs(internalControl) > 0) {
            shooter.spinwheel(internalControl);
        } else {
            if(button(ShooterInput.BUTTON)) {   
            
                shooter.shoot(shooterSpeed);
    
            } else {
               
                shooter.halt();
    
            }
        }
	
	}

    private final boolean button(ShooterInput intakeInput) {
        return inputMap.get(intakeInput).getAsBoolean();
    }

    private final double axis(ShooterAxis axis) {
		return axisMap.get(axis).getAsDouble();
	}

	@Override
	public void initSendable(SendableBuilder builder) {
		super.initSendable(builder);
		
	}
    
}
