package org.firstinspires.ftc.teamcode.Utils;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;

@Config
public class RelativeMotorAction implements Action {
    public static double TOLERANCE = 0.02; // revolutions

    private EnhancedMotor motor;
    private double power;
    private double dist; // degrees (0-360)

    private int targetPosTicks;
    private boolean firstLoop = true;

    public RelativeMotorAction(EnhancedMotor motor, double power, double dist) {
        this.motor = motor;
        this.power = power;
        this.dist = dist;
    }

    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        if (firstLoop) {
            targetPosTicks = motor.setTargetPositionRelative(dist);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor.setPower(dist > 0 ? power : -power); // if moving backwards, use negative power
            firstLoop = false;
        }
        int errorTicks = Math.abs(motor.motor.getCurrentPosition() - targetPosTicks);
        double errorRevs = errorTicks / motor.ticksPerRev;
        return errorRevs >= TOLERANCE; // true = keep going (out of tolerance), false = stop (in tolerance)
    }
}
