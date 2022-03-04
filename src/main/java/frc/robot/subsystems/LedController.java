package frc.robot.subsystems;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;

import frc.robot.RobotBase;

public class LedController extends SubsystemBase {
    private final I2C i2c;
    private final int blue = 0;
    private final int red = 1;

    DriverStation.Alliance color;
    private byte[] toSend = new byte[1];

    // blue = 0 red = 1
    public LedController(final RobotBase robot) {
        super(robot);
        i2c = new I2C(I2C.Port.kOnboard, 9);
        setTeamColor();
    }
    
    private void setTeamColor() {
		if (color == DriverStation.Alliance.Blue) {
            toSend[0] = 0;
            setColor();
        } 

        else if (color == DriverStation.Alliance.Red) {
            toSend[0] = 1;
            setColor();
        } 
        
        else {
            toSend[0] = 3; //whatever the default color we want
        }
    }

    private void setColor() {
        i2c.transaction(toSend, 1, null, 0);
    }
}