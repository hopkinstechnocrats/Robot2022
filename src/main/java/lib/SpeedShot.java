package lib;

import java.util.HashMap;

public class SpeedShot {
    public static double speed;
    public static final HashMap<Double, Double> speedMap = new HashMap<>();

    public SpeedShot() {

        speedMap.put(10.0, 2.0);//Distance, Speed
    }

    public double getLauncherDesiredSpeed(double distance) {
        speed = LinearInterpolation.getLinearInterpolation(speedMap, distance);
        return speed;
    }
}
