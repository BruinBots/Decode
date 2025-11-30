package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Autonomous.OpModes.IntakeAuto;
import org.firstinspires.ftc.teamcode.Components.AprilTags;
import org.firstinspires.ftc.teamcode.Components.Intake;
import org.firstinspires.ftc.teamcode.Components.Launcher;
import org.firstinspires.ftc.teamcode.Utils.ArtifactShakeAction;
import org.firstinspires.ftc.teamcode.Utils.ArtifactWaitAction;
import org.firstinspires.ftc.teamcode.Utils.BatteryVoltageCompensator;
import org.firstinspires.ftc.teamcode.Utils.ConditionalAction;
import org.firstinspires.ftc.teamcode.Utils.RelativeMotorAction;
import org.firstinspires.ftc.teamcode.Utils.ServoAction;
import org.firstinspires.ftc.teamcode.Utils.WaitAction;

public class MainBot {
    public DcMotorEx leftFrontMotor;
    public DcMotorEx rightFrontMotor;
    public DcMotorEx leftBackMotor;
    public DcMotorEx rightBackMotor;

    // Components
    public Launcher launcher;
    public Intake intake;

    public static MainBot shared;

    public MultipleTelemetry telemetry;
    public FtcDashboard dashboard = FtcDashboard.getInstance();
    public MecanumDrive drive;

    public HardwareMap hardwareMap;
    public AprilTags aprilTags;
    public BatteryVoltageCompensator voltageCompensator;

    public Gamepad launcherGamepad = null;

    public MainBot(HardwareMap hardwareMap, Telemetry telemetry) {
        leftFrontMotor = hardwareMap.get(DcMotorEx.class, "leftFront");
        rightFrontMotor = hardwareMap.get(DcMotorEx.class, "rightFront");
        leftBackMotor = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightBackMotor = hardwareMap.get(DcMotorEx.class, "rightBack");

        leftFrontMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBackMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Components
        launcher = new Launcher(hardwareMap);
        intake = new Intake(hardwareMap);

        this.telemetry = new MultipleTelemetry(dashboard.getTelemetry(), telemetry);
        this.hardwareMap = hardwareMap;

        aprilTags = new AprilTags(hardwareMap);

        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        voltageCompensator = new BatteryVoltageCompensator(hardwareMap);
    }

    public void moveBotMecanum(double drive, double rotate, double strafe, double scaleFactor) {
        double[] wheelSpeeds = new double[4];
        wheelSpeeds[0] = drive + strafe + rotate;  // left front
        wheelSpeeds[1] = drive - strafe - rotate;  // right front
        wheelSpeeds[2] = drive - strafe + rotate;  // left rear
        wheelSpeeds[3] = drive + strafe - rotate;  // right rear

        // finding the greatest power value
        double maxMagnitude = Math.max(Math.max(Math.max(wheelSpeeds[0], wheelSpeeds[1]), wheelSpeeds[2]), wheelSpeeds[3]);

        // dividing everyone by the max power value so that ratios are same (check if sdk automatically clips to see if go build documentation works
        if (maxMagnitude > 1.0) {
            for (int i = 0; i < wheelSpeeds.length; i++) {
                wheelSpeeds[i] /= maxMagnitude;
            }
        }

        // setting motor power and scaling down to preference
        leftFrontMotor.setPower(wheelSpeeds[0] * scaleFactor);
        rightFrontMotor.setPower(wheelSpeeds[1] * scaleFactor);
        leftBackMotor.setPower(wheelSpeeds[2] * scaleFactor);
        rightBackMotor.setPower(wheelSpeeds[3] * scaleFactor);
    }

    public Action singleLaunchActionBare(double power) {
        if (launcher.artifactPresent) {
            return new SequentialAction(
                    // Step 1: Spin up (async)
                    launcher.getPowerAction(power),

                    // Step 2: Intake to push an artifact into the launcher
                    // and get artifacts in a somewhat known position
//                    new RelativeMotorAction(intake.motor, Intake.SHORT_IN_POWER, Intake.SHORT_IN_DIST),

                    // Step 3: Outtake to ensure intake artifact isn't pushing against
                    // artifact in launcher
//                    new RelativeMotorAction(intake.motor, Intake.SHORT_OUT_POWER, -Intake.SHORT_OUT_DIST),
//                    intake.getPowerAction(0), // stop intake

                    // Step 4: Wait for spin up (conditional if artifact is now detected)
                    new ConditionalAction(() -> launcher.artifactPresent, new SequentialAction(
                            launcher.getVeloWaitAction(Launcher.MIN_LAUNCH_SPEED),
                            launcher.getAccelWaitAction(Launcher.MAX_LAUNCH_ACCEL),

                            // Step 5: Kick to launch
                            new RelativeMotorAction(intake.motor, Intake.INTAKE_POWER, Intake.SHORT_IN_DIST), // brief intake to prevent congestion in launcher
                            intake.getPowerAction(0), // stop intake
//                        launcher.kickAction()
                            new ServoAction(launcher.servo, Launcher.SERVO_UP_POS),
                            new WaitAction(Launcher.SERVO_WAIT_MS)
                    ))
            );
        } else {
            return telemetryPacket -> false;
        }
    }

    public Action singleLaunchAction(double power) {
        if (launcher.artifactPresent) {
            return new SequentialAction(
                    // Steps 1-5: Full launch
                    singleLaunchActionBare(power),

                    // Step 6: Intake to reload artifacts for next launch
                    new RelativeMotorAction(intake.motor, Intake.SHORT_IN_POWER, Intake.LONG_IN_DIST),
                    new RelativeMotorAction(intake.motor, Intake.REVERSE_POWER, -Intake.LONG_OUT_DIST),
                    intake.getPowerAction(0),

                    new ServoAction(launcher.servo, Launcher.SERVO_DOWN_POS),
                    new ArtifactWaitAction(Launcher.POST_LAUNCH_WAIT_MS)

                    // Step 7: Outtake to push the bottom artifact into the launcher
                    // (only if there were three artifacts inside before launching)
//                    new RelativeMotorAction(intake.motor, Intake.REVERSE_POWER, -Intake.LONG_OUT_DIST),
//                    intake.getPowerAction(0)
            );
        } else {
            return new Action() {
                @Override
                public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                    return false;
                }
            };
        }
    }

    public Action singleLaunchActionNoPreload(double power) {
        if (launcher.artifactPresent) {
            return new SequentialAction(
                    // Steps 1-5: Full launch
                    singleLaunchActionBare(power),
                    new ServoAction(launcher.servo, Launcher.SERVO_DOWN_POS),
                    new WaitAction(Launcher.SERVO_WAIT_MS)

                    // Step 7: Outtake to push the bottom artifact into the launcher
                    // (only if there were three artifacts inside before launching)
//                    new RelativeMotorAction(intake.motor, Intake.REVERSE_POWER, -Intake.LONG_OUT_DIST),
//                    intake.getPowerAction(0)
            );
        } else {
            return new Action() {
                @Override
                public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                    return false;
                }
            };
        }
    }

    public Action intakeDriveAction(boolean resetPose) {
        drive.updatePoseEstimate();
        if (resetPose) {
            drive.localizer.setPose(new Pose2d(0, 0, 0));
        }
        drive.updatePoseEstimate();
        return new SequentialAction(
                intake.getPowerAction(Intake.INTAKE_POWER),
                drive.actionBuilder(drive.localizer.getPose())
                        .lineToX(IntakeAuto.DISTANCE, new TranslationalVelConstraint(IntakeAuto.VELOCITY))
                        .build(),
                intake.getPowerAction(0)
        );
    }
}