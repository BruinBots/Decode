package org.firstinspires.ftc.teamcode.Components;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.MainBot;

@TeleOp(name="Obelisk Test", group="Testing")
public class ObeliskReaderTest extends OpMode {

    private ObeliskReader reader;

    @Override
    public void init() {
        MainBot.shared = new MainBot(hardwareMap, telemetry);
        reader = new ObeliskReader();
    }

    @Override
    public void loop() {
        telemetry.addData("Obelisk State", reader.read());
        telemetry.update();
    }
}
