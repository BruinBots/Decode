package org.firstinspires.ftc.teamcode.Components;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

import org.firstinspires.ftc.teamcode.MainBot;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

public class ObeliskReader {
    public enum ObeliskState {
        GPP,
        PGP,
        PPG,
        UNKNOWN,
    }

    private ObeliskState lastRead;

    public ObeliskReader() { }

    public ObeliskState read() {
        List<AprilTagDetection> detections = MainBot.shared.aprilTags.readAprilTags();

        for (AprilTagDetection detection: detections) {
            switch (detection.id) {
                case 21:
                    return ObeliskState.GPP;
                case 22:
                    return ObeliskState.PGP;
                case 23:
                    return ObeliskState.PPG;
            }
        }
        return ObeliskState.UNKNOWN;
    }

    public ObeliskState getLastRead() {
        return lastRead;
    }

    public class ObeliskReadAction implements Action {

        public ObeliskReader obeliskReader;

        private ObeliskReadAction(ObeliskReader obeliskReader) {
            this.obeliskReader = obeliskReader;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            ObeliskState state = obeliskReader.read();
            if (state == ObeliskState.UNKNOWN) { return true; }
            obeliskReader.lastRead = state;
            return false;
        }
    }

    public ObeliskReadAction getReadAction() {
        return new ObeliskReadAction(this);
    }
}
