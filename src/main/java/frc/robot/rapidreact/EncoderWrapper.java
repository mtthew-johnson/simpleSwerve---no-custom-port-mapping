package frc.robot.rapidreact;

import edu.wpi.first.wpilibj.DigitalSource;
import edu.wpi.first.wpilibj.Encoder;

public class EncoderWrapper extends Encoder {

    public double getprevious;
    public EncoderWrapper(DigitalSource sourceA, DigitalSource sourceB) {
        super(sourceA, sourceB);
    }
    

    public EncoderWrapper(int sourceA, int sourceB) {
        super(sourceA, sourceB);
    }

    // @Override
    // public double getDistance() {
    //     return super.getDistance() % 360;
    //   }

    public double getDistanceRaw() {
        return super.getDistance();
    }

    public double getDistanceWrapped() {
        return super.getDistance() % 360;

    }



    
}
