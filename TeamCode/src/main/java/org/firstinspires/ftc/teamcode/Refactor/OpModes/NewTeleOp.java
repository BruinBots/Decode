package org.firstinspires.ftc.teamcode.Refactor.OpModes;

import android.graphics.Color;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Autonomous.Constants;
import org.firstinspires.ftc.teamcode.Refactor.Jimmy;
import org.firstinspires.ftc.teamcode.Refactor.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Refactor.Subsystems.Shooter;

@TeleOp
@Configurable
public class NewTeleOp extends OpMode {

    private CommandScheduler m_scheduler;
    private Jimmy m_bot;
    private TelemetryManager telemetryM;

    private LynxModule controlHub;
    private LynxModule expansionHub;

    private Intake m_intake;
    private Shooter m_shooter;

    private GamepadEx m_gamepad1;
    private GamepadEx m_gamepad2;

    private Follower follower;
    private boolean slowMode = false;
    private boolean isRobotCentric = true;
    public static double SLOW_SPEED_MULTIPLIER = 0.5;

    @Override
    public void init() {
        m_scheduler = CommandScheduler.getInstance();

        m_bot = Jimmy.shared = new Jimmy(hardwareMap);
        m_intake = m_bot.getIntake();
        m_shooter = m_bot.getShooter();

        m_scheduler.registerSubsystem(m_intake, m_shooter);

        controlHub = m_bot.getControlHub();
        expansionHub = m_bot.getExpansionHub();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(0, 0, 0));
        follower.update();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        m_gamepad1 = new GamepadEx(gamepad1);
        m_gamepad2 = new GamepadEx(gamepad2);
    }

    @Override
    public void start() {
        follower.startTeleOpDrive();
    }

    @Override
    public void loop() {
        follower.update();
        telemetryM.update();

        if (m_gamepad1.wasJustPressed(GamepadKeys.Button.A)) {
            isRobotCentric = !isRobotCentric;
        }
        // Red if robot-centric, else teal
        controlHub.setConstant(isRobotCentric ? Color.rgb(1,0,0) : Color.rgb(0,1,1));
        telemetryM.debug("Robot Centric", isRobotCentric ? "YES" : "NO");

        if (m_gamepad1.isDown(GamepadKeys.Button.LEFT_STICK_BUTTON)) {
            slowMode = true;
        } else if (m_gamepad1.isDown(GamepadKeys.Button.RIGHT_STICK_BUTTON)) {
            slowMode = false;
        }
        // Yellow if slow mode, else purple
        expansionHub.setConstant(slowMode ? Color.rgb(1,1,0) : Color.rgb(1,0,1));
        telemetryM.debug("Slow Mode", slowMode ? "YES" : "NO");

        double driveMultiplier = slowMode ? SLOW_SPEED_MULTIPLIER : 1;
        follower.setTeleOpDrive(
                -gamepad1.left_stick_y * driveMultiplier,
                gamepad1.left_stick_x * driveMultiplier,
                gamepad1.right_stick_x * driveMultiplier,
                isRobotCentric
        );
        telemetryM.debug("position", follower.getPose());
        telemetryM.debug("velocity", follower.getVelocity());

        m_intake.doTelemetry(telemetryM);
        m_shooter.doTelemetry(telemetryM);

        m_gamepad1.readButtons();
        m_gamepad2.readButtons();

        m_scheduler.run();
    }
}
