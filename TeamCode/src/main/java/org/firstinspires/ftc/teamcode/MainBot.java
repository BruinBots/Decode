package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Components.AimBot;
import org.firstinspires.ftc.teamcode.Components.AimBotTester;
import org.firstinspires.ftc.teamcode.Components.Intake;
import org.firstinspires.ftc.teamcode.Components.Launcher;
import org.firstinspires.ftc.vision.VisionPortal;

public class MainBot {
//    public DcMotorEx leftFrontMotor;
//    public DcMotorEx rightFrontMotor;
//    public DcMotorEx leftBackMotor;
//    public DcMotorEx rightBackMotor;
//
//    // Components
//    public Launcher launcher;
//    public Intake intake;

    public static MainBot shared;
    public Telemetry telemetry;
    public HardwareMap hardwareMap;

    public VisionPortal visionPortal;


    public MainBot(HardwareMap hardwareMap, Telemetry telemetry) {
//        leftFrontMotor = hardwareMap.get(DcMotorEx.class, "left_front");
//        rightFrontMotor = hardwareMap.get(DcMotorEx.class, "right_front");
//        leftBackMotor = hardwareMap.get(DcMotorEx.class, "left_back");
//        rightBackMotor = hardwareMap.get(DcMotorEx.class, "right_back");
//
//        leftFrontMotor.setDirection(DcMotorSimple.Direction.REVERSE);
//        leftBackMotor.setDirection(DcMotorSimple.Direction.REVERSE);
//
//        // Components
//        launcher = new Launcher(hardwareMap);
//        intake = new Intake(hardwareMap);

        this.telemetry = telemetry;
        this.hardwareMap = hardwareMap;

        AimBot.initVisionPortal(hardwareMap);
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
//        leftFrontMotor.setPower(wheelSpeeds[0] * scaleFactor);
//        rightFrontMotor.setPower(wheelSpeeds[1] * scaleFactor);
//        leftBackMotor.setPower(wheelSpeeds[2] * scaleFactor);
//        rightBackMotor.setPower(wheelSpeeds[3] * scaleFactor);
    }
}