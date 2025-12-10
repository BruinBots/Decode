package org.firstinspires.ftc.teamcode.Refactor.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.SensorColor;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Refactor.Utils.TelemetryLogger;

@Configurable
public class Spindexer extends SubsystemBase {

    public static int INCREMENT = 500; // ticks between slots
    public static int LAUNCHER_POS = 800; // position to line up slot 0 with the launcher
    public static double TICKS_PER_REV = 145.1; // ticks per full revolution
    public static double POWER = 0.4;

    public static int[] BLACK = new int[]{0,0,0};
    public static int[] PURPLE = new int[]{255,0,255};
    public static int[] GREEN = new int[]{0,255,0};

    private MotorEx m_motor;
    private SensorColor m_sensor;

    private enum SlotState {
        UNKNOWN,
        EMPTY,
        PURPLE,
        GREEN
    }

    private enum SlotPosition {
        INTAKE,
        LAUNCHER,
    }

    private int curSlot;
    private SlotPosition curSlotPos;
    private SlotState[] slotStates;

    public Spindexer(HardwareMap hMap, String name) {
        m_motor = new MotorEx(hMap, name);
        m_sensor = new SensorColor(hMap, "colorSensor");

        m_motor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        m_motor.stopAndResetEncoder();

        curSlot = 0;
        curSlotPos = SlotPosition.INTAKE;
        slotStates = new SlotState[]{SlotState.UNKNOWN, SlotState.UNKNOWN, SlotState.UNKNOWN};
    }

    public void doTelemetry(TelemetryLogger telemetryM) {
        telemetryM.debug("Spindexer Slot", curSlot);
        telemetryM.debug("Spindexer Slot Pos", curSlotPos+(isMoving() ? " (moving)" : ""));
        telemetryM.debug("Spindexer Slots", slotStates[0]+","+slotStates[1]+","+slotStates[2]);
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

    public void moveSlotToIntake(int slot) {
        int pos = closestPos(posForIntake(slot));

        curSlot = slot;
        curSlotPos = SlotPosition.INTAKE;

        m_motor.setTargetPosition(pos);
        m_motor.setRunMode(Motor.RunMode.PositionControl);
        m_motor.set(POWER);
    }

    public void moveSlotToLauncher(int slot) {
        int pos = closestPos(posForLauncher(slot));

        curSlot = slot;
        curSlotPos = SlotPosition.LAUNCHER;

        m_motor.setTargetPosition(pos);
        m_motor.setRunMode(Motor.RunMode.PositionControl);
        m_motor.set(POWER);
    }

    public boolean isMoving() {
        return !m_motor.atTargetPosition();
    }

    private int[] toRGB(int[] argb) {
        return new int[]{ argb[1], argb[2], argb[3] };
    }

    private double rgbDistance(int[] rgb1, int[] rgb2) {
        int dr = rgb1[0] - rgb2[0];
        int dg = rgb1[1] - rgb2[1];
        int db = rgb1[2] - rgb2[2];
        return Math.sqrt(dr*dr + dg*dg + db*db);
    }

    private SlotState determineColor(int[] rgb) {
        double dblack  = rgbDistance(rgb, BLACK);
        double dpurple = rgbDistance(rgb, PURPLE);
        double dgreen  = rgbDistance(rgb, GREEN);

        double min = Math.min(dblack, Math.min(dpurple, dgreen));

        if (min == dblack)  return SlotState.EMPTY;
        if (min == dpurple) return SlotState.PURPLE;
        if (min == dgreen)  return SlotState.GREEN;

        return SlotState.UNKNOWN;
    }

    private void readSlotWithSensor() {
        if (curSlotPos == SlotPosition.INTAKE && !isMoving()) {
            int[] argb = m_sensor.getARGB();
            slotStates[curSlot] = determineColor(toRGB(argb));
        }
    }

    @Override
    public void periodic() {
        readSlotWithSensor();
    }
}
