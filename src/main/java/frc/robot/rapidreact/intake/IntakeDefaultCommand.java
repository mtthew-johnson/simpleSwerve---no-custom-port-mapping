package frc.robot.rapidreact.intake;

import java.util.Map;
import java.util.function.BooleanSupplier;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.rapidreact.intake.IntakeBase.IntakeInput;

public class IntakeDefaultCommand extends CommandBase {
    final Intake intake;
    final Map<IntakeInput, BooleanSupplier> inputMap;

    public IntakeDefaultCommand(final Intake intake, Map<IntakeInput, BooleanSupplier> inputMap) {
        this.intake = intake;
        this.inputMap = inputMap;

        addRequirements(intake);
        SendableRegistry.addChild(IntakeDefaultCommand.this, this);

    }

    @Override
	public void execute() {
        
        if(button(IntakeInput.BUTTON)) {   
            
            intake.intake();

        } else {
           
            intake.halt();

        }
	
	}

    private final boolean button(IntakeInput intakeInput) {
        return inputMap.get(intakeInput).getAsBoolean();
    }

	@Override
	public void initSendable(SendableBuilder builder) {
		super.initSendable(builder);
		
	}


}
