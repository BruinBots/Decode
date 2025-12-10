package org.firstinspires.ftc.teamcode.Refactor.OpModes;

import android.graphics.Color;

import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Autonomous.Constants;
import org.firstinspires.ftc.teamcode.Refactor.Commands.Intake.IntakeActivateCommand;
import org.firstinspires.ftc.teamcode.Refactor.Commands.Intake.IntakeReverseCommand;
import org.firstinspires.ftc.teamcode.Refactor.Commands.Shooter.ShooterKickCommand;
import org.firstinspires.ftc.teamcode.Refactor.Commands.Shooter.ShooterSpinWaitCommand;
import org.firstinspires.ftc.teamcode.Refactor.Commands.Shooter.ShooterStopCommand;
import org.firstinspires.ftc.teamcode.Refactor.Jimmy;
import org.firstinspires.ftc.teamcode.Refactor.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Refactor.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Refactor.Utils.TelemetryLogger;
import org.psilynx.psikit.core.Logger;
import org.psilynx.psikit.core.rlog.RLOGServer;
import org.psilynx.psikit.core.rlog.RLOGWriter;

@TeleOp
@Configurable
public class NewTeleOp extends CommandOpMode {

    private Jimmy m_bot;
    private TelemetryLogger m_telemetry;

    private LynxModule controlHub;
    private LynxModule expansionHub;

    private Intake m_intake;
    private Shooter m_shooter;

    private GamepadEx m_gamepad1;
    private GamepadEx m_gamepad2;

    private Follower follower;
    private boolean slowMode = false;
    public static double SLOW_SPEED_MULTIPLIER = 0.5;

    @Override
    public void initialize() {
        m_bot = Jimmy.shared = new Jimmy(hardwareMap);
        m_intake = m_bot.getIntake();
        m_shooter = m_bot.getShooter();

        register(m_intake, m_shooter);

        controlHub = m_bot.getControlHub();
        expansionHub = m_bot.getExpansionHub();

        follower = Constants.createFollower(hardwareMap);
//        follower.setStartingPose(new Pose(0, 0, 0));
//        follower.update();

        m_gamepad1 = new GamepadEx(gamepad1);
        m_gamepad2 = new GamepadEx(gamepad2);

        // Intake controls
        m_gamepad1.getGamepadButton(GamepadKeys.Button.A).whileHeld(new IntakeActivateCommand(m_intake));
        m_gamepad1.getGamepadButton(GamepadKeys.Button.B).whileHeld(new IntakeReverseCommand(m_intake));

        // Launcher controls
        m_gamepad1.getGamepadButton(GamepadKeys.Button.X).whileHeld(new ShooterSpinWaitCommand(m_shooter, Shooter.SPEED));
        m_gamepad1.getGamepadButton(GamepadKeys.Button.Y).whileHeld(new ShooterStopCommand(m_shooter));

        // Launcher kick controls
        m_gamepad1.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenPressed(new ShooterKickCommand(m_shooter));

        RLOGServer logserver = new RLOGServer();
        RLOGWriter logwriter = new RLOGWriter("NewTeleOp");
        Logger.addDataReceiver(logserver); // NOTE: Must be disabled during comp to be legal
        Logger.addDataReceiver(logwriter); // Comp legal
        m_telemetry = new TelemetryLogger(
                PanelsTelemetry.INSTANCE.getTelemetry(),
                telemetry
        );
        // Pull with:
        // adb pull /sdcard/FIRST/PsiKit/NewTeleOp.rlog
        // or
        // adb shell ls /sdcard/FIRST

        super.reset();

        follower.startTeleOpDrive();
    }

    @Override
    public void run() {
        super.run();
        m_telemetry.update();

        if (m_gamepad1.isDown(GamepadKeys.Button.LEFT_STICK_BUTTON)) {
            slowMode = true;
        } else if (m_gamepad1.isDown(GamepadKeys.Button.RIGHT_STICK_BUTTON)) {
            slowMode = false;
        }
        // Yellow if slow mode, else purple
        int color = slowMode ? Color.rgb(1,1,0) : Color.rgb(1,0,1);
        controlHub.setConstant(color);
        expansionHub.setConstant(color);
        m_telemetry.debug("Slow Mode", slowMode ? "YES" : "NO");

        double driveMultiplier = slowMode ? SLOW_SPEED_MULTIPLIER : 1;
        follower.setTeleOpDrive(
                -gamepad1.left_stick_y * driveMultiplier,
                gamepad1.left_stick_x * driveMultiplier,
                gamepad1.right_stick_x * driveMultiplier,
                true
        );
        follower.update();

        m_telemetry.debug("position", follower.getPose());
        m_telemetry.debug("velocity", follower.getVelocity());

        m_intake.doTelemetry(m_telemetry);
        m_shooter.doTelemetry(m_telemetry);

        m_gamepad1.readButtons();
        m_gamepad2.readButtons();
    }
}
