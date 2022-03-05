package frc.team5973.robot.rapidreact.commands.defaultCommands;

import java.util.Map;
import java.util.function.BooleanSupplier;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team5973.robot.rapidreact.Intake;
import frc.team5973.robot.rapidreact.Intake.IntakeInput;

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
        
        if(button(IntakeInput.COLLECT)) {   
            
            intake.extend();
            intake.collect();

        } else {
           
            intake.retract();
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
