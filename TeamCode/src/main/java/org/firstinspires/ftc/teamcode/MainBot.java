package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Autonomous.OpModes.IntakeAuto;
import org.firstinspires.ftc.teamcode.Components.AprilTags;
import org.firstinspires.ftc.teamcode.Components.Intake;
import org.firstinspires.ftc.teamcode.Components.Launcher;
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

    public Action singleLaunchAction(double power) {
        return new SequentialAction(
                launcher.getPowerAction(power),
                new WaitAction(Launcher.LAUNCH_WAIT_MS),
                launcher.getAccelWaitAction(Launcher.MAX_LAUNCH_ACCEL),
                launcher.kickAction(),
                new WaitAction(Launcher.POST_LAUNCH_WAIT_MS),
                intake.getPowerAction(Intake.INTAKE_POWER),
                new WaitAction(Intake.IN_WAIT_MS),
                intake.getPowerAction(Intake.REVERSE_POWER),
                new WaitAction(Intake.REVERSE_WAIT_MS),
                intake.getPowerAction(0)
        );
    }

    public Action intakeDriveAction() {
        drive.updatePoseEstimate();
        drive.localizer.setPose(new Pose2d(0, 0, 0));
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