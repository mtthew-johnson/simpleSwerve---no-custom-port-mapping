package frc.team5973.robot;

import edu.wpi.first.wpilibj.RobotBase;
import frc.team5973.robot.rapidreact.Robot;

public final class Main {
	
	private Main() {}

	public static void main(String[] args) {
		RobotBase.startRobot(Robot::new);
	}

}