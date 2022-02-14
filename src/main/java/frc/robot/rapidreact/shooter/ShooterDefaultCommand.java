package frc.robot.rapidreact.shooter;

import java.util.Map;
import java.util.function.BooleanSupplier;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.rapidreact.shooter.ShooterBase.ShooterInput;

public class ShooterDefaultCommand extends CommandBase {
    final Shooter shooter;
    final Map<ShooterInput, BooleanSupplier> inputMap;

    public ShooterDefaultCommand(final Shooter shooter, Map<ShooterInput, BooleanSupplier> inputMap) {
        this.shooter = shooter;
        this.inputMap = inputMap;

        addRequirements(shooter);
        SendableRegistry.addChild(ShooterDefaultCommand.this, this);
    }

    @Override
	public void execute() {
        
        if(button(ShooterInput.BUTTON)) {   
            
            shooter.shoot();

        } else {
           
            shooter.halt();

        }
	
	}

    private final boolean button(ShooterInput intakeInput) {
        return inputMap.get(intakeInput).getAsBoolean();
    }

	@Override
	public void initSendable(SendableBuilder builder) {
		super.initSendable(builder);
		
	}
    
}
