package org.firstinspires.ftc.teamcode.SBAs;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

//public class SBAAction implements Action {
//    private boolean initialized = false;
//    private SBA[] sbas;
//    private int curIdx = 0;
//
//    public SBAAction(SBA... sbas) {
//        this.sbas = sbas;
//    }
//
//    @Override
//    public boolean run(@NonNull TelemetryPacket packet) {
//        if (!initialized) {
//            if (!sbas[curIdx].sanity()) {
//                return true;
//            }
//            sbas[curIdx].init();
//        }
//
//        sbas[curIdx].loop();
//        if (!sbas[curIdx].isBusy()) {
//            curIdx += 1;
//            if (curIdx > sbas.length - 1) {
//                return true;
//            }
//        }
//        return false;
//    }
//}
