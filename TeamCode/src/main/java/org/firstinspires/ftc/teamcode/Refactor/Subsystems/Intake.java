package org.firstinspires.ftc.teamcode.Refactor.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Refactor.Commands.Intake.IntakeStopCommand;

@Configurable
public class Intake extends SubsystemBase {

    private final MotorEx motor;

    public static double POWER = 0.5;
    public static double REVERSE_POWER = 0.3;

    public Intake(final HardwareMap hMap, final String name) {
        motor = new MotorEx(hMap, name);
        motor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        motor.setRunMode(Motor.RunMode.RawPower);

        setDefaultCommand(new IntakeStopCommand(this));
    }

    public void doTelemetry(TelemetryManager telemetryM) {
        telemetryM.debug("Intake Power", motor.get());
    }

    public void activate() {
        motor.set(POWER);
    }

    public void reverse() {
        motor.set(REVERSE_POWER);
    }

    public void stop() {
        motor.set(0);
    }
}
