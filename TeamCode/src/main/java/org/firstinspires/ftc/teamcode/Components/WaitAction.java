package org.firstinspires.ftc.teamcode.Components;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

public class WaitAction implements Action {
    private int waitMs;
    private long startTime = 0;

    public WaitAction(int waitMs) {
        this.waitMs = waitMs;
    }

    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        if (startTime == 0) {
            startTime = System.currentTimeMillis();
            return true;
        }
        return System.currentTimeMillis() <= startTime + waitMs;
    }
}
