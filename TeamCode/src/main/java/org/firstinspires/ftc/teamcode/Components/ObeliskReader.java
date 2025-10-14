package org.firstinspires.ftc.teamcode.Components;

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
}
