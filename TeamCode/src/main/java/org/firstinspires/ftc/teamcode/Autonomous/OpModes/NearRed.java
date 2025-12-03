package org.firstinspires.ftc.teamcode.Autonomous.OpModes;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.Arclength;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.PoseMap;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "NearRed")
public class NearRed extends NearAuto {

    @Override
    public Pose2d getStartPose() {
        return new Pose2d(60, -36, Math.toRadians(90));
    }

    @Override
    public PoseMap getPoseMap() {
        return new PoseMap() {
            @NonNull
            @Override
            public Pose2dDual<Arclength> map(@NonNull Pose2dDual<Arclength> pose2dDual) {
                return new Pose2dDual<Arclength>(pose2dDual.position.x, pose2dDual.position.y.unaryMinus(), pose2dDual.heading.inverse());
            }
        };
    }
}
