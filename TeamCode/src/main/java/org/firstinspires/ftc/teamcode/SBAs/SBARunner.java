package org.firstinspires.ftc.teamcode.SBAs;

import org.firstinspires.ftc.teamcode.MainBot;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

//public class SBARunner {
//    public ArrayList<SBA> curSBAs = new ArrayList<>();
//    public int curIdx;
//    public int curInitIdx;
//
//    public void runSBAs(SBA... sbas) {
//        runSBAs(sbas, true);
//    }
//
//    public void runSBAs(SBA[] sbas, boolean overwrite) {
//        if (overwrite || curSBAs == null) {
//            curSBAs = new ArrayList<>(Arrays.asList(sbas));
//            curIdx = 0;
//            curInitIdx = -1;
//        } else {
//            curSBAs.addAll(Arrays.asList(sbas));
//        }
//    }
//
//    public void stop() {
//        curSBAs.clear();
//    }
//
//    public boolean isBusy() {
//        return curSBAs.size() > 0;
//    }
//
//    public void loop() {
//        // Ensure we're currently running an SBA list
//        if (curSBAs.size() == 0 || curIdx >= curSBAs.size()) {
//            MainBot.shared.telemetry.addData("SBA", "Exiting "+curSBAs.size()+", curIdx="+curIdx);
//            return;
//        }
//        SBA sba = curSBAs.get(curIdx);
//        MainBot.shared.telemetry.addData("SBA", "Running "+sba+", curIdx="+curIdx+", curInitIdx="+curInitIdx);
//        if (curInitIdx < curIdx) {
//            sba.preInit();
//            if (!sba.sanity()) {
//                stop();
//            } // Quit if sanity check fails
//            sba.init();
//            curInitIdx = curIdx;
//        }
//        sba.loop();
//        if (!sba.isBusy()) { // move onto next SBA if done with current one
//            curIdx++;
//            if (curIdx >= curSBAs.size()) {
//                stop();
//            } // End SBA list if curIdx gets past the list limit
//        }
//    }
//
//    // TODO: SBA Stubs down here
//    /*
//    Example:
//    public void runArmUp() {
//        runSBAs(new MotorSBA(bot.armMotor, Arm.POWER, Arm.TOP_POS));
//    }
//     */
//}
