package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Components.Intake;
import org.firstinspires.ftc.teamcode.Components.Launcher;
import org.firstinspires.ftc.teamcode.Components.VelMotor;
import org.firstinspires.ftc.teamcode.SBAs.SBARunner;

@TeleOp(name="Spin up SBA Test", group="Testing")
public class SpinUpSBATest extends OpMode {
    SBARunner runner;
    Intake vMotor;

    boolean didSpinUpLast = true;

    @Override
    public void init() {
        vMotor = new Intake(hardwareMap);
        runner = new SBARunner();
        runner.runSBAs(vMotor.getSpinUpSBA());
        MainBot.shared = new MainBot(hardwareMap, telemetry);
    }

    @Override
    public void loop() {
        vMotor.doTelemetry();
        runner.loop();
        if (!runner.isBusy()) {
            if (didSpinUpLast) {
                runner.runSBAs(vMotor.getStopSBA());
                didSpinUpLast = false;
            } else {
                runner.runSBAs(vMotor.getSpinUpSBA());
                didSpinUpLast = true;
            }
        }
    }
}
