package com.example.meepmeeptesting;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

public class WaitAction implements Action {
    private int waitMs;
    private long startTime = 0;

    public WaitAction(int waitMs) {
        this.waitMs = waitMs;
    }

    @Override
    public boolean run(TelemetryPacket telemetryPacket) {
        if (startTime == 0) {
            startTime = System.currentTimeMillis();
            return true;
        }
        return System.currentTimeMillis() <= startTime + waitMs;
    }
}
