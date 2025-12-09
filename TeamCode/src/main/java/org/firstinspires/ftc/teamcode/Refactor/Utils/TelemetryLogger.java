package org.firstinspires.ftc.teamcode.Refactor.Utils;

import com.bylazar.telemetry.TelemetryManager;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.psilynx.psikit.core.Logger;

public class TelemetryLogger {
    private final TelemetryManager telemetryM;
    private final Telemetry telemetry;

    public TelemetryLogger(TelemetryManager telemetryM, Telemetry telemetry) {
        this.telemetryM = telemetryM;
        this.telemetry = telemetry;
    }

    public void debug(String key, Object v) {
        String val = v.toString();
        telemetryM.debug(key, val);
        telemetry.addData(key, val);
        Logger.recordOutput(key, val);
    }

    public void addLine(String line) {
        telemetryM.addLine(line);
        telemetry.addLine(line);
        Logger.logInfo(line);
    }

    public void update() {
        telemetryM.update();
        telemetry.update();
    }
}
