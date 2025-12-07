package org.firstinspires.ftc.teamcode.Utils;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.hardware.lynx.commands.core.LynxGetPWMEnableResponse;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.ftccommon.internal.manualcontrol.exceptions.FailedToOpenHubException;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Components.Launcher;
import org.firstinspires.ftc.teamcode.MainBot;

import java.util.ArrayList;

@Config
public class EnhancedMotor {
    public DcMotorEx motor;
    public double ticksPerRev;

    public static double VEL_INTERVAL = 150;
    public static double ACCEL_INTERVAL = 500;

    public EnhancedMotor(DcMotorEx motor) {
        this.motor = motor;
    }

    public EnhancedMotor(HardwareMap hardwareMap, String name) {
        motor = hardwareMap.get(DcMotorEx.class, name);
    }

    public void setTicksPerRev(double ticksPerRev) {
        this.ticksPerRev = ticksPerRev;
    }

    public void setPower(double power) {
        motor.setPower(MainBot.shared.voltageCompensator.getAdjustedPower(power));
    }

    public void setDirection(DcMotorSimple.Direction direction) {
        motor.setDirection(direction);
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior) {
        motor.setZeroPowerBehavior(behavior);
    }

    public void setMode(DcMotor.RunMode mode) {
        motor.setMode(mode);
    }

    public int setTargetPositionRelative(double dist) { // set target position (relative) in units of rotation (1 = one full rotation)
        int curPosTicks = motor.getCurrentPosition();
        int deltaPosTicks = (int)(dist * ticksPerRev);
        int targetPosTicks = curPosTicks + deltaPosTicks;
        motor.setTargetPosition((int)targetPosTicks);
        return targetPosTicks;
    }

    private double lastVelTime = 0;
    private double lastVel = 0;
    private double lastPosTime = 0;
    private double lastPos = 0;
    private double vel = 0;
    private double accel = 0;

    // Velocity PID
    private double targetVelocityRPM = 0;
    public static double kP = 0.0007;
    public static double kI = 0;
    public static double kD = 0.00001;

    // multiply all target velocities by this amount
    // (compensation for improperly tuned or absent kI term)
    public static double VELOCITY_ADJUSTMENT = 1.26;

    private double integral = 0;
    private double lastError = 0;

    private double lastOutputPower = 0;
    public static double MAX_POWER_DELTA = 0.01; // max change per call
    // tolerance within target
    // in which MAX_POWER_DELTA is enforced
    public static double MAX_POWER_DELTA_BAND = 250;

    private ArrayList<Double> derivativeRunningAverage = new ArrayList<>();
    public double lastDerivativeAverage;
    public static int DERIVATIVE_RUNNING_AVERAGE_SIZE = 10;

    public ArrayList<Double> pastVelocities = new ArrayList<>();
    public static int PAST_VELOCITIES_SIZE = 50;

    public double getRPM() {
        // Update accel
        double curVel = motor.getVelocity() * 60.0 / ticksPerRev;
        double curTime = System.currentTimeMillis();

        if (curTime - lastVelTime > ACCEL_INTERVAL) {
            double dv = curVel - lastVel;
            double dt = curTime - lastVelTime;
            accel = dv / dt;

            lastVel = curVel;
            lastVelTime = curTime;
        }

        // Smoothed velocity
        double curPos = motor.getCurrentPosition() * 60.0 / ticksPerRev; // revs
        if (curTime - lastPosTime > VEL_INTERVAL) {
            double dx = curPos - lastPos;
            double dt = (curTime - lastPosTime) / 1000.0;
            vel = dx / dt;

            lastPos = curPos;
            lastPosTime = curTime;
        }

        pastVelocities.add(vel);
        if (pastVelocities.size() > PAST_VELOCITIES_SIZE) {
            pastVelocities.remove(0);
        }

        return vel;
    }

    public double getAcceleration() {
        return accel;
    }

    public double getPower() {
        return motor.getPower();
    }

    public void setVelPID(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }

    public void resetVPIDIntegral() {
        integral = 0;
    }

    public void setTargetVelocity(double rpm) {
        this.targetVelocityRPM = rpm*VELOCITY_ADJUSTMENT;
        derivativeRunningAverage.clear();
        lastDerivativeAverage = 99999;
        integral = 0;
        lastError = 0;
    }

    public void updateVelocityPID() {
        if (targetVelocityRPM == 0) {
//            setPower(0);
            return;
        }

        double currentRPM = getRPM();
        double error = targetVelocityRPM - currentRPM;
        integral += error;
        double derivative = error - lastError;
        derivativeRunningAverage.add(derivative);
        if (derivativeRunningAverage.size() > DERIVATIVE_RUNNING_AVERAGE_SIZE) {
            derivativeRunningAverage.remove(0);
        }
        double sum = 0;
        for (double i: derivativeRunningAverage) {
            sum += i;
        }
        lastDerivativeAverage = sum / derivativeRunningAverage.size();

        double output = kP * error + kI * integral + kD * derivative;

        // Clamp output
        output = Math.max(0, Math.min(1, output));

        // Limit sudden power changes
        if (Math.abs(error) < MAX_POWER_DELTA_BAND) {
            double delta = output - lastOutputPower;
            if (delta > MAX_POWER_DELTA) delta = MAX_POWER_DELTA;
            else if (delta < -MAX_POWER_DELTA) delta = -MAX_POWER_DELTA;
            output = lastOutputPower + delta;
        }

        setPower(output);

        lastOutputPower = output;
        lastError = error;
    }
}
