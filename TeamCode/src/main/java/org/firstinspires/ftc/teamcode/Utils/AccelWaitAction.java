package org.firstinspires.ftc.teamcode.Utils;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

public class AccelWaitAction implements Action {
    private EnhancedMotor motor;
    private double maxAccel;

    public AccelWaitAction(EnhancedMotor motor, double maxAccel) {
        this.motor = motor;
        this.maxAccel = maxAccel;
    }

    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        double accel = motor.getAcceleration();
        return accel > maxAccel;
    }
}