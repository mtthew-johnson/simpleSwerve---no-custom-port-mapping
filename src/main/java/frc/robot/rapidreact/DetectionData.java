package frc.robot.rapidreact;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
public class DetectionData {
    
    public NetworkTable resolution = NetworkTableInstance.getDefault().getTable("Resolution");

    public NetworkTable blueBalls = NetworkTableInstance.getDefault().getTable("Blue Balls");
    public NetworkTable redBalls = NetworkTableInstance.getDefault().getTable("Red Balls");
    public NetworkTable robots = NetworkTableInstance.getDefault().getTable("Robots");

    public NetworkTableEntry resX;
	public NetworkTableEntry resY;

    public NetworkTableEntry blueBallCenterX;
	public NetworkTableEntry blueBallCenterY;

    public NetworkTableEntry redBallCenterX;
	public NetworkTableEntry redBallCenterY;

    public NetworkTableEntry blueBallBoxWidth;
	public NetworkTableEntry blueBallBoxHeight;

    public NetworkTableEntry redBallBoxWidth;
	public NetworkTableEntry redBallBoxHeight;

    public NetworkTableEntry isBlueBallDetected;
    public NetworkTableEntry isRedBallDetected;

    private double distanceFromTarget;
    
    public double distanceFromTarget(String ballType) {
		// // distance constant divided by length between centers of contours
		// distanceFromTarget = DISTANCE_CONSTANT / getCenterX();
		
		double width = 9.5; //inches // width of the ball

        if(ballType.equals("blue")) {

            blueBallBoxWidth = blueBalls.getEntry("Box Width");
            double boxWidthentry = blueBallBoxWidth.getDouble(-1);
            distanceFromTarget = (width * 333.82) / boxWidthentry; // 333.82 is the focal length of the microsoft lifecam in px
            
        } else if(ballType.equals("red")) {

            redBallBoxWidth = redBalls.getEntry("Box Width");
            double boxWidthentry = redBallBoxWidth.getDouble(-1);
            distanceFromTarget = (width * 333.82) / boxWidthentry; // 333.82 is the focal length of the microsoft lifecam in px
            
        }

        return distanceFromTarget;

	}
    
    public double getResX() {
		
        resX = resolution.getEntry("Width");
		
		return resX.getDouble(-1);

	}

	public double getResY() {
		
        resX = resolution.getEntry("Height");
		
		return resY.getDouble(-1);

	}

    public double getBlueBallCenterX() {
        
        blueBallCenterX = blueBalls.getEntry("centerX");
        
        return blueBallCenterX.getDouble(-1);

    }

    public double getBlueBallCenterY() {
        
        blueBallCenterY = blueBalls.getEntry("centerY");
        
        return blueBallCenterY.getDouble(-1);

    }

    public double getRedBallCenterX() {
        
        redBallCenterX = redBalls.getEntry("centerX");
        
        return redBallCenterX.getDouble(-1);

    }

    public double getRedBallCenterY() {
        
        redBallCenterY = redBalls.getEntry("centerY");
        
        return redBallCenterY.getDouble(-1);

    }

    public double getBlueBallBoxWidth() {
        
        blueBallBoxWidth = blueBalls.getEntry("Box Width");

        return blueBallBoxWidth.getDouble(-1);

    }

    public double getBlueBallBoxHeight() {
       
        blueBallBoxHeight = blueBalls.getEntry("Box Height");

        return blueBallBoxHeight.getDouble(-1);

    }

    public double getRedBallBoxWidth() {
        
        redBallBoxWidth = redBalls.getEntry("Box Width");

        return redBallBoxWidth.getDouble(-1);

    }

    public double getRedBallBoxHeight() {
        
        redBallBoxHeight = redBalls.getEntry("Box Height");

        return redBallBoxHeight.getDouble(-1);

    }

    public boolean isBlueBallDetected() {
        isBlueBallDetected = blueBalls.getEntry("Blue Ball in Frame");
        
        return isBlueBallDetected.getBoolean(true);
    }
    
    public boolean isRedBallDetected() {
        
        isRedBallDetected = redBalls.getEntry("Red Ball in Frame");
        
        return isRedBallDetected.getBoolean(true);

    }
}
