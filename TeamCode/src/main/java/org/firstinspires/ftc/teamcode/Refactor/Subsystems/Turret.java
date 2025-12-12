package org.firstinspires.ftc.teamcode.Refactor.Subsystems;

import android.health.connect.datatypes.units.Power;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import com.seattlesolvers.solverslib.util.MathUtils;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Refactor.Jimmy;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@Configurable
public class Turret extends SubsystemBase {
    /*
    Turret with camera mounted on it
     */

    private MotorEx m_motor;
    private VisionPortal m_visionportal;
    private AprilTagProcessor m_apriltag;

    private Double m_aprilpos = null;
    private Double m_aprildist = null;
    private Timer m_apriltimer;

    private PIDFController m_controller;

    public static PIDFCoefficients pidf_coeffs = new PIDFCoefficients(0,0,0,0);
    public static double MAX_APRIL_ABSENT_DELAY = 0.75; // seconds

    public static double MIN_POS = -250;
    public static double MAX_POS = 250;
    public static double SEEK_POWER = 0.3;
    public static double MAX_POWER_AMPLITUDE = 0.7;
    public static double TOLERANCE = 5; // inches

    public Turret(HardwareMap hMap) {
        m_motor = new MotorEx(hMap, "turretMotor");
        m_motor.setRunMode(Motor.RunMode.RawPower);
        m_motor.setDistancePerPulse(1 / 537.7); // GoBilda 5203 312RPM motor

        m_apriltag = AprilTagProcessor.easyCreateWithDefaults();
        m_visionportal = VisionPortal.easyCreateWithDefaults(hMap.get(WebcamName.class, "Webcam 1"), m_apriltag);

        m_controller = new PIDFController(pidf_coeffs.p, pidf_coeffs.i, pidf_coeffs.d, pidf_coeffs.f);
        m_apriltimer = new Timer();
    }

    private void readAprilTag() {
        List<AprilTagDetection> detections = m_apriltag.getDetections();
        int goalID = (Jimmy.shared.alliance == Jimmy.Alliance.BLUE) ? 20 : 24;
        for (AprilTagDetection detection: detections) {
            if (detection.metadata != null) {
                if (detection.metadata.id == goalID) { // found goal
                    m_aprilpos = detection.ftcPose.x;
                    m_aprildist = detection.ftcPose.y;
                    m_apriltimer.resetTimer();
                }
            }
        }
        if (m_apriltimer.getElapsedTimeSeconds() > MAX_APRIL_ABSENT_DELAY) {
            m_aprilpos = null;
            m_aprildist = null;
        }
    }

    public boolean isLocked() {
        return m_aprilpos != null;
    }

    public boolean isAimed() {
        return Math.abs(m_aprilpos) < TOLERANCE;
    }

    @Override
    public void periodic() {
        m_visionportal.resumeStreaming();

        readAprilTag();
        m_controller.setPIDF(pidf_coeffs.p, pidf_coeffs.i, pidf_coeffs.d, pidf_coeffs.f);

        double power = 0;
        double curPos = m_motor.getCurrentPosition();

        if (curPos >= MAX_POS) {
            power = -SEEK_POWER; // seek back in bounds
        } else if (curPos <= MIN_POS) {
            power = SEEK_POWER; // seek back in bounds
        }

        if (m_aprilpos != null) { // april tag found
            // PIDF control loop
            double calc = MathUtils.clamp(m_controller.calculate(m_aprilpos, 0), -MAX_POWER_AMPLITUDE, MAX_POWER_AMPLITUDE);
            if (curPos >= MAX_POS) {
                if (calc < 0) {
                    power = calc;
                } else {
                    // reset controller
                    m_controller.reset();
                }
            } else if (curPos <= MIN_POS) {
                if (calc > 0) {
                    power = calc;
                } else {
                    // reset controller
                    m_controller.reset();
                }
            } else {
                power = calc;
            }
        } else {
            // reset controller; we lost the tag
            m_controller.reset();
        }

        m_motor.set(power);
    }
}
