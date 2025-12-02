package org.firstinspires.ftc.teamcode.Utils;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;
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
    public static double FULL_BATTERY_VOLTAGE = 13.35;

    private VoltageSensor sensor;
    public double lastVoltage;
    private DcMotorEx[] motors;

    public BatteryVoltageCompensator(HardwareMap hardwareMap) {
        sensor = hardwareMap.get(VoltageSensor.class, "Control Hub");
        lastVoltage = getCurrentVoltage();
        motors = new DcMotorEx[]{
                MainBot.shared.leftFrontMotor,
                MainBot.shared.leftBackMotor,
                MainBot.shared.rightBackMotor,
                MainBot.shared.rightFrontMotor,
                MainBot.shared.launcher.motor.motor,
                MainBot.shared.intake.motor.motor,
                MainBot.shared.lifter.motor.motor
        };
    }

    public double getCurrentVoltage() {
        return sensor.getVoltage();
    }

    private void updateVoltageIfNoMotors() {
        for (DcMotorEx motor: motors) {
            if (motor.getPower() != 0) {
                return;
            }
        }
        lastVoltage = getCurrentVoltage();
    }

    public void doTelemetry() {
        updateVoltageIfNoMotors();
        MainBot.shared.telemetry.addData("Battery Voltage", lastVoltage+"/"+FULL_BATTERY_VOLTAGE+"V");
    }

    public double getAdjustedPower(double origPower) {
        if (!ENABLED) { return origPower; }
        double adjustedPower = origPower * (FULL_BATTERY_VOLTAGE / lastVoltage); // adjust power
        if (adjustedPower > 1) { return 1; }
        return adjustedPower;
    }
}
