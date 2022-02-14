package frc.robot.subsystems;

import edu.wpi.first.wpilibj.ADIS16470_IMU;
import frc.robot.RobotBase;

public class Gyro extends SubsystemBase {

    private ADIS16470_IMU gyro;

    public Gyro(RobotBase robot) {
        super(robot);

        gyro = new ADIS16470_IMU();
    }

    public void calibrate() {
        gyro.calibrate();
    }

    public void close() {
        gyro.close();
    }

    public int configCalTime(ADIS16470_IMU.CalibrationTime new_cal_time) {
        return gyro.configCalTime(new_cal_time);
    }

    public double getAccelX() {
        return gyro.getAccelX();
    }

    public double getAccelY() {
        return gyro.getAccelY();
    }

    public double getAccelZ() {
        return gyro.getAccelZ();
    }

    public double getAngle() {
        return gyro.getAngle();
    }

    public int getPort() {
        return gyro.getPort();
    }

    public ADIS16470_IMU.IMUAxis getYawAxis() {
        return gyro.getYawAxis();
    }

    public double getYComplementaryAngle() {
        return gyro.getYComplementaryAngle();
    }

    public double getYFilteredAccelAngle() {
        return gyro.getYFilteredAccelAngle();
    }

    public void reset() {
        gyro.reset();
    }

    public int setYawAxis(ADIS16470_IMU.IMUAxis yaw_axis) {
        return gyro.setYawAxis(yaw_axis);
    }
    
}
