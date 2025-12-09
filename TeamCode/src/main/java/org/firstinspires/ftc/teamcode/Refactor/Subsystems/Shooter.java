package org.firstinspires.ftc.teamcode.Refactor.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.Refactor.Commands.Shooter.ShooterStopAction;

@Configurable
public class Shooter extends SubsystemBase {

    private final MotorEx motor;

    public static PIDFCoefficients VELO_COEFFS = new PIDFCoefficients(
            0.,
            0.,
            0.,
            0.
    );

    public Shooter(final HardwareMap hMap, final String name) {
        motor = new MotorEx(hMap, name);
        motor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        motor.setRunMode(Motor.RunMode.VelocityControl);

        setDefaultCommand(new ShooterStopAction(this));
    }

    public void doTelemetry(TelemetryManager telemetryM) {
        telemetryM.debug("Shooter Power", motor.get());
        telemetryM.debug("Shooter Velocity", getSpeed());
        telemetryM.debug("Shooter Acceleration", motor.getAcceleration());
    }

    public void setSpeed(double speed) {
        motor.setRunMode(Motor.RunMode.VelocityControl);
        motor.setVelocity(speed);
    }

    public void stop(boolean brake) {
        motor.setZeroPowerBehavior(brake ? Motor.ZeroPowerBehavior.BRAKE : Motor.ZeroPowerBehavior.FLOAT);
        motor.setRunMode(Motor.RunMode.RawPower);
        motor.set(0);
    }

    public double getSpeed() {
        return motor.getCorrectedVelocity();
    }

    @Override
    public void periodic() {
        // Update PIDF from dash
        motor.setVeloCoefficients(VELO_COEFFS.p, VELO_COEFFS.i, VELO_COEFFS.d);
        motor.setFeedforwardCoefficients(0., VELO_COEFFS.f);
    }
}
