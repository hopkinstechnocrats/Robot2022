package lib.motorcontroller;

import com.ctre.phoenix.motorcontrol.can.BaseMotorController;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import lib.talonconfiguration.BaseTalonFXConfiguration;
import lib.talonconfiguration.BaseTalonSRXConfiguration;

import java.util.HashMap;
import java.util.Map;

public class MotorControllerFactory {
    private static Map<Integer, BaseMotorController> mockFalcons = new HashMap<>();
    private static final Map<Integer, BaseMotorController> mockTalonSRXs = new HashMap<>();

    public static void setMotorControllerMock(MotorType motorType, int CANID, BaseMotorController mock) {
        if (motorType == MotorType.FALCON) {
            mockFalcons.put(CANID, mock);
        } else if (motorType == MotorType.TALONSRX) {
            mockTalonSRXs.put(CANID, mock);
        }
    }

    public static void clearAllMocks() {
        mockFalcons = new HashMap<>();
        mockFalcons = new HashMap<>();
    }

    public static BaseMotorController getMotorController(MotorType motorType, int CANID) {
        if (motorType == MotorType.FALCON) {
            if (mockFalcons.get(CANID) != null) {
                return mockFalcons.get(CANID);
            }
            WPI_TalonFX motor = new WPI_TalonFX(CANID);
            motor.configAllSettings(new BaseTalonFXConfiguration());
            return motor;
        } else if (motorType == MotorType.TALONSRX) {
            if (mockTalonSRXs.get(CANID) != null) {
                return mockTalonSRXs.get(CANID);
            }
            WPI_TalonSRX motor = new WPI_TalonSRX(CANID);
            motor.configAllSettings(new BaseTalonSRXConfiguration());
            return motor;
        }
        return null;
    }

    enum MotorType {
        FALCON,
        TALONSRX
    }
}
