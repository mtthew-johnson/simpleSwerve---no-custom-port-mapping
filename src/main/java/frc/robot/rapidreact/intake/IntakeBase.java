package frc.robot.rapidreact.intake;

import frc.robot.RobotBase;
import frc.robot.subsystems.SubsystemBase;

public class IntakeBase extends SubsystemBase {

    public IntakeBase(final RobotBase robot) {
        super(robot);
        
    }

    public IntakeBase(String name, final RobotBase robot) {
        super(name, robot);
    }

    public enum IntakeInput {BUTTON}

    @Override
	public Class<? extends SubsystemBase> getEffectiveClass() {
		return IntakeBase.class;
	}

	@Override
	public String getConfigName() {
		return "intake";
	}
    
}
