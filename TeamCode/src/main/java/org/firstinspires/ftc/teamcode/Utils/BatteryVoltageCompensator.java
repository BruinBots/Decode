package org.firstinspires.ftc.teamcode.Utils;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.MainBot;

@Config
public class BatteryVoltageCompensator {
    /*
    Helper class to compensate for battery voltage
    As battery voltage drops, motors run slower.
    This will adjust the motor power accordingly
    to ensure the motors stay running at about the
    same speed as full battery.
     */

    public static boolean ENABLED = true;
    public static double FULL_BATTERY_VOLTAGE = 12.8;

    private VoltageSensor sensor;

    public BatteryVoltageCompensator(HardwareMap hardwareMap) {
        sensor = hardwareMap.get(VoltageSensor.class, "Control Hub");
    }

    public double getCurrentVoltage() {
        return sensor.getVoltage();
    }

    public void doTelemetry() {
        MainBot.shared.telemetry.addData("Battery Voltage", getCurrentVoltage()+"/"+FULL_BATTERY_VOLTAGE+"V");
    }

    public double getAdjustedPower(double origPower) {
        if (!ENABLED) { return origPower; }
        double adjustedPower = origPower * (FULL_BATTERY_VOLTAGE / getCurrentVoltage()); // adjust power
        if (adjustedPower > 1) { return 1; }
        return adjustedPower;
    }
}
