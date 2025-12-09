package org.firstinspires.ftc.teamcode.Refactor.Utils;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Refactor.Jimmy;
import org.firstinspires.ftc.teamcode.Refactor.Subsystems.Shooter;
import org.psilynx.psikit.core.Logger;
import org.psilynx.psikit.core.rlog.RLOGServer;
import org.psilynx.psikit.core.rlog.RLOGWriter;

import java.util.ArrayList;

@Config
@TeleOp
public class ShooterCurveTuner extends LinearOpMode {
    /*
    Runs launcher at various powers, then writes
    resulting RPM to a file for further processing
    to create a curve of rpm vs. power for more
    accurate launcher control
     */

    public static String TEST_POWERS = "0.1,0.2,0.3";
    public static double STABILIZE_TIME = 10; // seconds

    private TelemetryLogger m_telemetry;

    @Override
    public void runOpMode() throws InterruptedException {
        Jimmy bot = Jimmy.shared = new Jimmy(hardwareMap);

        ArrayList<Double> testPowers = new ArrayList<>();
        for (String strPower: TEST_POWERS.split(",")) {
            testPowers.add(Double.parseDouble(strPower));
        }

        RLOGServer logserver = new RLOGServer();
        RLOGWriter logwriter = new RLOGWriter("NewTeleOp");
        Logger.addDataReceiver(logserver); // NOTE: Must be disabled during comp to be legal
        Logger.addDataReceiver(logwriter); // Comp legal
        m_telemetry = new TelemetryLogger(
                PanelsTelemetry.INSTANCE.getTelemetry(),
                telemetry
        );

        waitForStart();

        m_telemetry.addLine("Will test "+testPowers.size()+" powers");
        m_telemetry.update();

        ArrayList<String> data = new ArrayList<>();

        Shooter shooter = bot.getShooter();

        for (double power: testPowers) {
            shooter.setPower(power);
            Timer timer = new Timer();
            timer.resetTimer();
            while (timer.getElapsedTimeSeconds() < STABILIZE_TIME) {
                shooter.doTelemetry(m_telemetry);
                m_telemetry.update();
                wait(10); // short 10ms delay
            }
            double speed = shooter.getSpeed();
            m_telemetry.debug("Test "+power, power+","+speed);
        }

        m_telemetry.update();
    }
}