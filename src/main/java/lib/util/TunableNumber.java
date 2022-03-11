package lib.util;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class TunableNumber {
    private static final String tableKey = "TunableNumbers";
    private static boolean tuningMode = false;

    private final String key;
    private final double defaultValue;

    public TunableNumber(String dashboardKey, double defaultValue) {
        this.key = tableKey + "/" + dashboardKey;
        this.defaultValue = defaultValue;
        if (tuningMode) {
            SmartDashboard.putNumber(key, SmartDashboard.getNumber(key, defaultValue));
        } else {
            SmartDashboard.delete(key);
        }
    }

    public double get() {
        return tuningMode ? SmartDashboard.getNumber(key, defaultValue) : defaultValue;
    }

    public static void setTuningMode(boolean isTunable) {
        tuningMode = isTunable;
    }

}
