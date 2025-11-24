package org.firstinspires.ftc.teamcode.Utils;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

public class VeloWaitAction implements Action {
    private EnhancedMotor motor;
    private double minVel;

    public VeloWaitAction(EnhancedMotor motor, double minVel) {
        this.motor = motor;
        this.minVel = minVel;
    }

    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        double vel = motor.getRPM();
        telemetryPacket.addLine("VeloWaitAction "+vel+"("+minVel+")");
        return vel < minVel;
    }
}
