package org.firstinspires.ftc.teamcode.Utils;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

@Config
public class VeloWaitAction implements Action {
    private EnhancedMotor motor;
    private double velTarget;
    public static double TOLERANCE = 50; // rpm

    public VeloWaitAction(EnhancedMotor motor, double velTarget) {
        this.motor = motor;
        this.velTarget = velTarget;
    }

    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        motor.setTargetVelocity(velTarget);
        telemetryPacket.addLine("VeloWaitAction ("+velTarget+"RPM)");
        return Math.abs(motor.getRPM() - velTarget) > TOLERANCE;
    }
}
