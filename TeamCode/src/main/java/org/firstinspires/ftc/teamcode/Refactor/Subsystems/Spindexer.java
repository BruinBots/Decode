package org.firstinspires.ftc.teamcode.Refactor.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.SensorColor;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Configurable
public class Spindexer extends SubsystemBase {

    public static int INCREMENT = 500; // ticks between slots
    public static int LAUNCHER_POS = 800; // position to line up slot 0 with the launcher
    public static double TICKS_PER_REV = 145.1; // ticks per full revolution
    public static double POWER = 0.4;

    private MotorEx m_motor;
    private SensorColor m_sensor;

    public Spindexer(HardwareMap hMap, String name) {
        m_motor = new MotorEx(hMap, name);
        m_sensor = new SensorColor(hMap, "colorSensor");

        m_motor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        m_motor.stopAndResetEncoder();
    }

    private int posForIntake(int slot) {
        return INCREMENT * slot;
    }

    private int posForLauncher(int slot) {
        return posForIntake(slot) + LAUNCHER_POS;
    }

    private int closestPos(int pos) {
        int cur = m_motor.getCurrentPosition();
        return (int)(cur + ((pos - cur + TICKS_PER_REV/2.) % TICKS_PER_REV) - TICKS_PER_REV/2.);
        // Validated with Python algorithm:
        // cur + ((pos - cur + ppr/2) % ppr) - ppr/2
    }

    // TODO: State machine (currently selecting)
    // TODO: State per slot (empty, purple, green, unknown)
    // TODO: Slot reader

    public void moveSlotToIntake(int slot) {
        int pos = closestPos(posForIntake(slot));
        m_motor.setTargetPosition(pos);
        m_motor.setRunMode(Motor.RunMode.PositionControl);
        m_motor.set(POWER);
    }

    public void moveSlotToLauncher(int slot) {
        int pos = closestPos(posForLauncher(slot));
        m_motor.setTargetPosition(pos);
        m_motor.setRunMode(Motor.RunMode.PositionControl);
        m_motor.set(POWER);
    }
}
