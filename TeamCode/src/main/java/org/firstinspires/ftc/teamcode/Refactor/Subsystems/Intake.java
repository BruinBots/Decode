package org.firstinspires.ftc.teamcode.Refactor.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Refactor.Commands.Intake.IntakeStopCommand;
import org.firstinspires.ftc.teamcode.Refactor.Utils.TelemetryLogger;

@Configurable
public class Intake extends SubsystemBase {

    private final MotorEx m_motor;

    public static double POWER = 0.5;
    public static double REVERSE_POWER = 0.3;

    public Intake(final HardwareMap hMap, final String name) {
        m_motor = new MotorEx(hMap, name);
        m_motor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        m_motor.setRunMode(Motor.RunMode.RawPower);

        setDefaultCommand(new IntakeStopCommand(this));
    }

    public void doTelemetry(TelemetryLogger telemetryM) {
        telemetryM.debug("Intake Power", m_motor.get());
    }

    public void activate() {
        m_motor.set(POWER);
    }

    public void reverse() {
        m_motor.set(REVERSE_POWER);
    }

    public void stop() {
        m_motor.set(0);
    }
}
