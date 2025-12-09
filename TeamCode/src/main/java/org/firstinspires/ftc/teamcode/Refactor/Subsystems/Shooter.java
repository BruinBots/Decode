package org.firstinspires.ftc.teamcode.Refactor.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.Refactor.Commands.Shooter.ShooterStopCommand;
import org.firstinspires.ftc.teamcode.Refactor.Utils.TelemetryLogger;

@Configurable
public class Shooter extends SubsystemBase {

    private final MotorEx m_motor;
    private final SimpleServo m_servo;
    private Timer stateTimer;

    public static PIDFCoefficients VELO_COEFFS = new PIDFCoefficients(
            0.,
            0.,
            0.,
            0.
    );

    private enum ShooterState {
        IDLE,
        SPIN_UP,
        LAUNCH,
        RESET
    }
    private ShooterState state = ShooterState.IDLE;

    private int shotsRemaining = 0;

    // Spin up constants
    private double targetSpeed = 0;
    public static double SPEED_TOLERANCE = 50; // ticks/s
    public static double MAX_SPIN_UP_TIME = 4.; // seconds

    // Kick constants
    public static double KICK_DOWN_POS = 90; // degrees
    public static double KICK_UP_POS = 151; // degrees
    public static double KICK_UP_DURATION = 0.4; // seconds
    public static double KICK_DOWN_DURATION = 0.35; // seconds

    public Shooter(final HardwareMap hMap, final String name) {
        m_motor = new MotorEx(hMap, name);
        m_motor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        m_motor.setRunMode(Motor.RunMode.VelocityControl);

        m_servo = new SimpleServo(hMap, "kickServo", KICK_DOWN_POS, KICK_UP_POS);

        stateTimer.resetTimer();

        setDefaultCommand(new ShooterStopCommand(this));
    }

    public void doTelemetry(TelemetryLogger telemetryM) {
        telemetryM.debug("Shooter Power", m_motor.get());
        telemetryM.debug("Shooter Velocity", getSpeed());
        telemetryM.debug("Shooter Acceleration", m_motor.getAcceleration());
        telemetryM.debug("Shooter State", state);
    }

    public void setSpeed(double speed) {
        m_motor.setRunMode(Motor.RunMode.VelocityControl);
        m_motor.setVelocity(speed);
        targetSpeed = speed;
    }

    public void stop(boolean brake) {
        m_motor.setZeroPowerBehavior(brake ? Motor.ZeroPowerBehavior.BRAKE : Motor.ZeroPowerBehavior.FLOAT);
        m_motor.setRunMode(Motor.RunMode.RawPower);
        m_motor.set(0);
        targetSpeed = 0;
    }

    public double getSpeed() {
        return m_motor.getCorrectedVelocity();
    }

    public void fireShots(int numShots) {
        if (state == ShooterState.IDLE) {
            shotsRemaining = numShots;
        }
    }

    public boolean isBusy() {
        return state != ShooterState.IDLE;
    }

    @Override
    public void periodic() {
        // State machine
        switch (state) {
            case IDLE:
                if (shotsRemaining > 0 && targetSpeed != 0) {
                    stateTimer.resetTimer();
                    state = ShooterState.SPIN_UP;
                }
                break;
            case SPIN_UP:
                double error = Math.abs(targetSpeed - getSpeed());
                if (error < SPEED_TOLERANCE || stateTimer.getElapsedTimeSeconds() > MAX_SPIN_UP_TIME) {
                    m_servo.setPosition(KICK_UP_POS);
                    stateTimer.resetTimer();
                    state = ShooterState.LAUNCH;
                }
                break;
            case LAUNCH:
                if (stateTimer.getElapsedTimeSeconds() > KICK_UP_DURATION) {
                    m_servo.setPosition(KICK_DOWN_POS);
                    stateTimer.resetTimer();
                    state = ShooterState.RESET;
                }
                break;
            case RESET:
                if (stateTimer.getElapsedTimeSeconds() > KICK_DOWN_DURATION) {
                    shotsRemaining --;
                    stateTimer.resetTimer();
                    if (shotsRemaining > 0) {
                        // spin up
                        state = ShooterState.SPIN_UP;
                    } else {
                        state = ShooterState.IDLE;
                    }
                }
                break;
        }

        // Update PIDF from dash
        m_motor.setVeloCoefficients(VELO_COEFFS.p, VELO_COEFFS.i, VELO_COEFFS.d);
        m_motor.setFeedforwardCoefficients(0., VELO_COEFFS.f);
    }
}
