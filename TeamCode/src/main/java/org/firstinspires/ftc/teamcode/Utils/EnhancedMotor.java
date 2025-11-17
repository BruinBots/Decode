package org.firstinspires.ftc.teamcode.Utils;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.hardware.lynx.commands.core.LynxGetPWMEnableResponse;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.ftccommon.internal.manualcontrol.exceptions.FailedToOpenHubException;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Components.Launcher;

@Config
public class EnhancedMotor {
    public DcMotorEx motor;
    public double ticksPerRev;

    public static double ACCEL_INTERVAL = 500;

    public EnhancedMotor(DcMotorEx motor) {
        this.motor = motor;
    }

    public EnhancedMotor(HardwareMap hardwareMap, String name) {
        motor = hardwareMap.get(DcMotorEx.class, name);
    }

    public void setTicksPerRev(double ticksPerRev) {
        this.ticksPerRev = ticksPerRev;
    }

    public void setPower(double power) {
        motor.setPower(power);
    }

    public void setDirection(DcMotorSimple.Direction direction) {
        motor.setDirection(direction);
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior) {
        motor.setZeroPowerBehavior(behavior);
    }

    public void setMode(DcMotor.RunMode mode) {
        motor.setMode(mode);
    }

    private double lastVelTime = 0;
    private double lastVel = 0;
    private double accel = 0;

    public double getRPM() {
        double curVel = motor.getVelocity() * 60.0 / ticksPerRev;
        double curTime = System.currentTimeMillis();

        if (curTime - lastVelTime > ACCEL_INTERVAL) {
            double dv = curVel - lastVel;
            double dt = curTime - lastVelTime;
            accel = dv / dt;

            lastVel = curVel;
            lastVelTime = curTime;
        }

        return curVel;
    }

    public double getAcceleration() {
        return accel;
    }

    public double getPower() {
        return motor.getPower();
    }
}
