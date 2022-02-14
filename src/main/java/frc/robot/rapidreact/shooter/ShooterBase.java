package frc.robot.rapidreact.shooter;

import frc.robot.RobotBase;
import frc.robot.subsystems.SubsystemBase;

public class ShooterBase extends SubsystemBase {

    public ShooterBase(final RobotBase robot) {
        super(robot);
    }

    public ShooterBase(String name, final RobotBase robot) {
        super(name, robot);
    }

    public enum ShooterInput {BUTTON}

    @Override
	public Class<? extends SubsystemBase> getEffectiveClass() {
		return ShooterBase.class;
	}

	@Override
	public String getConfigName() {
		return "shooter";
	}
}
