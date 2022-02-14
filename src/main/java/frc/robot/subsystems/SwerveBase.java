package frc.robot.subsystems;

import frc.robot.RobotBase;

public abstract class SwerveBase extends SubsystemBase {
    private double wheelDiameter = 6.0;
    
	public SwerveBase(final RobotBase robot) {
        super(robot);
    }

    public SwerveBase(String name, final RobotBase robot) {
        super(name, robot);
    }

    public enum Input {FORWARD, STRAFE, TURN}

    @Override
	public Class<? extends SubsystemBase> getEffectiveClass() {
		return SwerveBase.class;
	}

	@Override
	public String getConfigName() {
		return "swerveDrive";
	}

	public void swerveDrive(double forwardValue, double strafeValue, double rotateValue) {
		swerveDrive(forwardValue, strafeValue, rotateValue, true);
	}

	public abstract void swerveDrive(double forwardValue, double strafeValue, double rotateValue, boolean squareInputs); 

	public double getWheelDiameter() {
		return wheelDiameter;
	}

	public void setWheelDiameter(double diameter) {
		this.wheelDiameter = diameter;
	}

	public double getWheelCircumference() {
		return getWheelDiameter() * Math.PI;
	}
    public enum SwerveModule {FRONT_LEFT, FRONT_RIGHT, BACK_LEFT, BACK_RIGHT}
}
