package org.firstinspires.ftc.teamcode.Autonomous;

public class BaseAuto {
//    private MecanumDrive drive;
//
//    private MainBot bot;
//    private AimBot aimBot;
//    private ObeliskReader obeliskReader;
//
//    public BaseAuto() {
//        drive = new MecanumDrive(MainBot.shared.hardwareMap, getStartPose());
//        bot = MainBot.shared;
//        aimBot = new AimBot();
//        obeliskReader = new ObeliskReader();
//    }
//
//    public Pose2d getStartPose() {
//        return new Pose2d(-60, 12, 0);
//    }
//
//    public class TrajectoryEnd {
//        public Action action;
//        public TrajectoryActionBuilder builder;
//
//        public TrajectoryEnd(Action action, TrajectoryActionBuilder builder) {
//            this.action = action;
//            this.builder = builder;
//        }
//    }
//
//    public TrajectoryEnd launch(Pose2d startPose, boolean isFirstLaunch) {
//        TrajectoryActionBuilder builder = drive.actionBuilder(startPose)
//                .afterTime(0, bot.launcher.getSpinUpAction(Launcher.LAUNCH_SPEED)); // Launcher.LAUNCH_POWER,
//        if (isFirstLaunch) {
//                builder = builder
//                        .splineTo(new Vector2d(-42, -36), Math.toRadians(225));
//        } else {
//            builder = builder
//                    .splineToLinearHeading(new Pose2d(-42, -36, Math.toRadians(225)), Math.toRadians(225));
//        }
//        return new TrajectoryEnd(
//                new SequentialAction(
//                        builder.build(),
//                        new ParallelAction(
//                                aimBot.getAction(),
//                                bot.launcher.getSpinUpAction(Launcher.LAUNCH_SPEED) // Launcher.LAUNCH_POWER,
//                        ),
//                        bot.launcher.getServoAction(Launcher.SERVO_UP_POS),
//                        new WaitAction(1000),
//                        bot.launcher.getServoAction(Launcher.SERVO_DOWN_POS)
//                ),
//                builder.endTrajectory()
//        );
//    }
//
//    public TrajectoryEnd obeliskRead(Pose2d startPose) {
//        TrajectoryActionBuilder builder = drive.actionBuilder(startPose)
//                .setReversed(true)
//                .lineToYLinearHeading(-18, Math.toRadians(-20))
//                .setReversed(false);
//        return new TrajectoryEnd(
//                new SequentialAction(
//                        builder.build(),
//                        obeliskReader.getReadAction()
//                ),
//                builder.endTrajectory()
//        );
//    }

    // Intake paths
//    public TrajectoryEnd GPPIntake(TrajectoryActionBuilder builder) {
//        builder = builder
//                .afterDisp(1, bot.intake.getServoAction(Intake.INTAKE_IN_POS))
//                .splineTo(new Vector2d(36, -36), Math.toRadians(270))
//                .lineToY(-48)
//                .setReversed(true)
//                .splineToLinearHeading(new Pose2d(18, -18, Math.toRadians(180)), Math.toRadians(180))
//                .setReversed(false)
//                .splineTo(new Vector2d(-42, -36), Math.toRadians(225));
//        return new TrajectoryEnd(
//                builder.build(),
//                builder
//        );
//    }
//
//    public TrajectoryEnd PGPIntake(TrajectoryActionBuilder builder) {
//        builder = builder
//                .afterDisp(1, bot.intake.getServoAction(Intake.INTAKE_IN_POS))
//                .splineTo(new Vector2d(-12, -36), Math.toRadians(270))
//                .lineToY(-48)
//                .setReversed(true)
//                .splineToLinearHeading(new Pose2d(-42, -36, Math.toRadians(225)), Math.toRadians(225))
//                .setReversed(false);
//        return new TrajectoryEnd(
//                builder.build(),
//                builder
//        );
//    }
//
//    public TrajectoryEnd PPGIntake(TrajectoryActionBuilder builder) {
//        builder = builder
//                .afterDisp(1, bot.intake.getServoAction(Intake.INTAKE_IN_POS))
//                .splineTo(new Vector2d(12, -36), Math.toRadians(270))
//                .lineToY(-48)
//                .setReversed(true)
//                .splineToLinearHeading(new Pose2d(0, -18, Math.toRadians(180)), Math.toRadians(180))
//                .setReversed(false)
//                .splineTo(new Vector2d(-42, -36), Math.toRadians(225));
//        return new TrajectoryEnd(
//                builder.build(),
//                builder
//        );
//    }
}
