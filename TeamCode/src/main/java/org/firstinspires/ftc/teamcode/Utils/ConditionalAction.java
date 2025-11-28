package org.firstinspires.ftc.teamcode.Utils;

import android.media.MediaParser;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

import java.util.function.BooleanSupplier;

public class ConditionalAction implements Action {
    private BooleanSupplier condition;
    private Action action;
    private boolean firstLoop = true;

    public ConditionalAction(BooleanSupplier condition, Action action) {
        this.condition = condition;
        this.action = action;
    }

    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        if (firstLoop) {
            firstLoop = false;
            if (condition.getAsBoolean()) {
                return action.run(telemetryPacket);
            } else {
                return false;
            }
        }
        return action.run(telemetryPacket);
    }
}
