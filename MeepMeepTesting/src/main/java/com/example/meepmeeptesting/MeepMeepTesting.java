package com.example.meepmeeptesting;

import static java.lang.Math.PI;
import static java.lang.Math.abs;
import static java.lang.Math.atan2;
import static java.lang.Math.max;
import static java.lang.Math.min;
import static java.lang.Math.toRadians;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.util.Arrays;

public class MeepMeepTesting {

    public static double
            DEAD_TIME = 0.5,
            LENGTH_ROBOT = 14.2,
            WIDTH_ROBOT = 14.2,
            SIZE_HALF_FIELD = 70.5,
            SIZE_TILE = 23.625,
            DISTANCE_BETWEEN_SPECIMENS = 2,
            DISTANCE_FROM_BASKET_SWEEP = 30,
            EXTEND_SAMPLE_1 = 21,
            EXTEND_SAMPLE_2 = 20,
            EXTEND_SAMPLE_3 = 20,
            EXTEND_OVER_SUB_BAR_1 = 50 / 25.4 + 1.5,
            EXTEND_OVER_SUB_BAR_2 = 50 / 25.4 + 1.5,
            PRE_EXTEND_1 = 8,
            PRE_EXTEND_2 = 0,
            PRE_EXTEND_3 = 8,
            TIME_EXTEND = 0.6,
            TIME_RETRACT = .75,
            WAIT_RE_SWEEP = 1,
            ANGLE_PRE_SLAM = 0.1,
            WAIT_PRE_SLAM_BUCKET = 0.25,
            ANGLE_SLAMMING = 1.1,
            WAIT_SLAMMING_BUCKET = 0.35,
            SPEED_SLAMMING = -0.75,
            SPEED_INTAKING = 1,
            SPEED_EXTEND = 1,
            SPEED_RETRACT = -0.6,
            SPEED_SPIKE_TURNING = 2,
            SPEED_SWEEPING_SUB = 6.5,
            SPEED_SWEEPING_SUB_TURNING = 0.5,
            SPEED_INCHING = 5,
            SPEED_INCHING_TURNING = 0.75,
            WAIT_SCORE_SAMPLE_PRELOAD = 1.5,
            WAIT_APPROACH_WALL = 0,
            WAIT_APPROACH_BASKET = 0,
            WAIT_APPROACH_CHAMBER = 0,
            WAIT_POST_INTAKING = 0,
            WAIT_SCORE_BASKET = 0.25,
            WAIT_SCORE_CHAMBER = 0.1,
            WAIT_DROP_TO_EXTEND = 0.2,
            WAIT_INTAKE_RETRACT_POST_SUB = 0,
            WAIT_EXTEND_MAX_SPIKE = 3,
            WAIT_SWEEPER_EXTEND = 0.3,
            WAIT_SWEEPER_RETRACT = 0,
            LIFT_HEIGHT_TOLERANCE = 3.75,
            X_OFFSET_CHAMBER_1 = 1,
            X_OFFSET_CHAMBER_2 = -1,
            X_OFFSET_CHAMBER_3 = -2,
            X_OFFSET_CHAMBER_4 = -3,
            Y_INCHING_FORWARD_WHEN_INTAKING = 10,
            TIME_CYCLE = 5,
            TIME_SCORE = 0.5;

    /// <a href="https://www.desmos.com/calculator/l8pl2gf1mb">Adjust spikes 1 and 2</a>
    /// <a href="https://www.desmos.com/calculator/sishohvpwc">Visualize spike samples</a>
    public static EditablePose
            admissibleError = new EditablePose(1, 1, toRadians(2)),
            admissibleVel = new EditablePose(25, 25, toRadians(30)),

            intakingPartnerSample = new EditablePose(-29,7 - SIZE_HALF_FIELD, 0),

            intaking1 = new EditablePose(-61, -54, PI/3),
            intaking2 = new EditablePose(-62, -52, 1.4632986527692424),
            intaking3 = new EditablePose(-61, -50, 2 * PI / 3),

            sample1 = new EditablePose(-48, -26.8, PI / 2),
            sample2 = new EditablePose(-58, -27.4, PI / 2),
            sample3 = new EditablePose(-68.5, -27.8, PI / 2),

            park1 = new EditablePose(-0.5 * SIZE_TILE, -2 * SIZE_TILE, PI),
            park2 = new EditablePose(-1.5 * SIZE_TILE, -2 * SIZE_TILE, PI / 2),
            park3 = new EditablePose(-2.5 * SIZE_TILE, -2 * SIZE_TILE, PI / 2),

            snapshotPos = new EditablePose(-25, -10, toRadians(20)),

            basket2 = new EditablePose(-63, -54, 1),
            basket3 = new EditablePose(-54.5, -54.5, PI / 4),

            sub1 = new EditablePose(-22.5, -11, 0),
            sub2 = new EditablePose(sub1.x, -4, 0),

            basketFromSub = new EditablePose(-57.25, -57.25, PI/4),

            aroundBeamPushing = new EditablePose(35, -30, PI / 2),

            chamberRight = new EditablePose(0.5 * WIDTH_ROBOT + 0.375, -33, - PI / 2),
            chamberLeft = new EditablePose(-chamberRight.x, -33, PI / 2),

            pushing1 = new EditablePose(46, -13, toRadians(-80)),
            pushing2 = new EditablePose(57, pushing1.y, toRadians(-70)),
            pushing3 = new EditablePose(63, pushing1.y, - PI / 2),

            intakingSpec = new EditablePose(36, -60, PI / 2),

            pushed1 = new EditablePose(pushing1.x, -46, toRadians(110)),
            pushed2 = new EditablePose(pushing2.x, pushed1.y, toRadians(110)),
            pushed3 = new EditablePose(pushing3.x, intakingSpec.y, - PI / 2);

    static Pose2d pose = new Pose2d(0,0, 0.5 * PI);
    static boolean isRedAlliance = false;

    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(720);

        pose = new Pose2d(0.5 * LENGTH_ROBOT + 0.375 - 2 * SIZE_TILE, 0.5 * WIDTH_ROBOT - SIZE_HALF_FIELD, 0);

        intaking1.heading = intaking1.angleTo(sample1);
        intaking2.heading = intaking2.angleTo(sample2);
        intaking3.heading = intaking3.angleTo(sample3);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(50, 50, 5.3, 20, 15.649408396574804)
                .setDimensions(WIDTH_ROBOT, LENGTH_ROBOT)
                .setStartPose(pose)
                .build();

        TrajectoryActionBuilder builder = myBot.getDrive().actionBuilder(new Pose2d(0.5 * LENGTH_ROBOT + 0.375 - 2 * SIZE_TILE, 0.5 * WIDTH_ROBOT - SIZE_HALF_FIELD, 0));

        EditablePose sub1Edited = sub1.clone(), sub2Edited = sub2.clone();

        AngularVelConstraint spikeConstraint = new AngularVelConstraint(SPEED_SPIKE_TURNING);

        MinVelConstraint inchingConstraint = new MinVelConstraint(Arrays.asList(
                new TranslationalVelConstraint(SPEED_INCHING),
                new AngularVelConstraint(SPEED_INCHING_TURNING)
        ));

        MinVelConstraint sweepConstraint = new MinVelConstraint(Arrays.asList(
                new TranslationalVelConstraint(SPEED_SWEEPING_SUB),
                new AngularVelConstraint(SPEED_SWEEPING_SUB_TURNING)
        ));

        System.out.println(intaking1.angleTo(sample1) - intaking1.heading); // -9.73490512395 deg
        System.out.println(intaking1.angleTo(sample2) - intaking1.heading); // -32.336758263 deg

        System.out.println(intaking1.distTo(sample1));
        System.out.println(intaking1.distTo(sample2));

        builder = builder
//                .strafeToLinearHeading(intaking1.toVector2d(), intaking1.heading)
//                .stopAndAdd(scoreSample())
//
//                .afterTime(0, () -> {
////                    robot.intake.setRollerAndAngle(SPEED_INTAKING);
////                    robot.intake.extendo.setTarget(EXTEND_SAMPLE_1);
////                    timer.reset();
//                })
////                .stopAndAdd(telemetryPacket -> !(timer.seconds() >= WAIT_EXTEND_MAX_SPIKE || robot.intake.hasSample() || robot.intake.extendo.atPosition(EXTEND_SAMPLE_1)))
//                .setTangent(intaking1.heading)
//                .lineToY(intaking1.y + Y_INCHING_FORWARD_WHEN_INTAKING, inchingConstraint)
//
//                .stopAndAdd(finishIntaking())
//                .strafeToLinearHeading(intaking2.toVector2d(), intaking2.heading)
//                .stopAndAdd(scoreSample())
//
//                .afterTime(0, () -> {
////                    robot.intake.setRollerAndAngle(SPEED_INTAKING);
////                    robot.intake.extendo.setTarget(EXTEND_SAMPLE_2);
////                    timer.reset();
//                })
////                .stopAndAdd(telemetryPacket -> !(timer.seconds() >= WAIT_EXTEND_MAX_SPIKE || robot.intake.hasSample() || robot.intake.extendo.atPosition(EXTEND_SAMPLE_2)))
//                .setTangent(intaking2.heading)
//                .lineToY(intaking2.y + Y_INCHING_FORWARD_WHEN_INTAKING, inchingConstraint)
//
////                    .afterTime(0, preExtend(robot, PRE_EXTEND_3))
//                .stopAndAdd(finishIntaking())
////                    .afterTime(0, () -> robot.intake.sweeper.setActivated(true))
//                .strafeToLinearHeading(basket2.toVector2d(), basket2.heading)
////                    .afterTime(0, () -> robot.intake.sweeper.setActivated(false))
//                .stopAndAdd(scoreSample())
//
//                .strafeToLinearHeading(intaking3.toVector2d(), intaking3.heading, spikeConstraint)
//                .afterTime(0, () -> {
////                    robot.intake.setRollerAndAngle(SPEED_INTAKING);
////                    robot.intake.extendo.setTarget(EXTEND_SAMPLE_3);
////                    timer.reset();
//                })
////                .stopAndAdd(telemetryPacket -> !(timer.seconds() >= WAIT_EXTEND_MAX_SPIKE || robot.intake.hasSample() || robot.intake.extendo.atPosition(EXTEND_SAMPLE_3)))
//                .setTangent(PI / 2)
//                .lineToY(intaking3.y + Y_INCHING_FORWARD_WHEN_INTAKING, inchingConstraint)
//
//                .stopAndAdd(finishIntaking())
////                    .afterTime(0, () -> robot.intake.sweeper.setActivated(true))
//                .strafeToLinearHeading(basket3.toVector2d(), basket3.heading)
////                    .afterTime(0, () -> robot.intake.sweeper.setActivated(false))
//                .stopAndAdd(scoreSample())
//
////                .afterTime(0, () -> robot.intake.extendo.setTarget(EXTEND_OVER_SUB_BAR_1))
//                .afterDisp(DISTANCE_FROM_BASKET_SWEEP, sweep())
//                .setTangent(basket3.heading)
//                .splineTo(sub1Edited.toVector2d(), sub1Edited.heading)
//                .stopAndAdd(approachSub())
//
//                .setTangent(PI / 2)
//                .lineToY(-sub1.y, sweepConstraint)
//
//                .stopAndAdd(finishIntaking())
//                .stopAndAdd(new SequentialAction(
////                        new InstantAction(() -> robot.intake.setAngle(ANGLE_PRE_SLAM)),
////                        new SleepAction(WAIT_PRE_SLAM_BUCKET),
////                        new InstantAction(() -> robot.intake.setRoller(SPEED_SLAMMING)),
////                        new InstantAction(() -> robot.intake.setAngle(ANGLE_SLAMMING)),
////                        new SleepAction(WAIT_SLAMMING_BUCKET),
////                        new InstantAction(() -> robot.intake.setRollerAndAngle(0))
//                ))
//                .setTangent(PI)
//                .waitSeconds(WAIT_INTAKE_RETRACT_POST_SUB)
//                .splineTo(basketFromSub.toVector2d(), PI + basketFromSub.heading)
//                .stopAndAdd(scoreSample())
//
////                .afterTime(0, () -> robot.intake.extendo.setTarget(EXTEND_OVER_SUB_BAR_1))
//                .afterDisp(DISTANCE_FROM_BASKET_SWEEP, sweep())
//                .setTangent(basket3.heading)
//                .splineTo(sub1Edited.toVector2d(), sub1Edited.heading)
//                .stopAndAdd(approachSub())

                .setTangent(PI / 2)
                .strafeToSplineHeading(snapshotPos.toVector2d(), snapshotPos.heading)

//                .stopAndAdd(finishIntaking())
//                .stopAndAdd(new SequentialAction(
////                        new InstantAction(() -> robot.intake.setAngle(ANGLE_PRE_SLAM)),
////                        new SleepAction(WAIT_PRE_SLAM_BUCKET),
////                        new InstantAction(() -> robot.intake.setRoller(SPEED_SLAMMING)),
////                        new InstantAction(() -> robot.intake.setAngle(ANGLE_SLAMMING)),
////                        new SleepAction(WAIT_SLAMMING_BUCKET),
////                        new InstantAction(() -> robot.intake.setRollerAndAngle(0))
//                ))
                .setTangent(PI + snapshotPos.heading)
//                .waitSeconds(WAIT_INTAKE_RETRACT_POST_SUB)
                .splineTo(basketFromSub.toVector2d(), PI + basketFromSub.heading)
//                .stopAndAdd(scoreSample())
//                .strafeToLinearHeading(park3.toVector2d(), park1.heading)
                .setTangent(basketFromSub.heading)
                .splineTo(park2.toVector2d(), 0)
//                .strafeTo(park1.toVector2d())
                ;



        myBot.runAction(
                builder
                        .build()
        );

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }

    private static Action scoreSample() {
        return new SequentialAction(
                new InstantAction(() -> {
//                    if (!robot.hasSample()) robot.intake.transfer(NEUTRAL);
                }),
                new SleepAction(WAIT_APPROACH_BASKET),
//                telemetryPacket -> !(robot.deposit.basketReady() && abs(robot.deposit.lift.getPosition() - HEIGHT_BASKET_HIGH) <= LIFT_HEIGHT_TOLERANCE),
//                new InstantAction(robot.deposit::nextState),
                new SleepAction(WAIT_SCORE_BASKET)
        );
    }

    private static Action scoreSpecimen() {
        return new SequentialAction(
                new InstantAction(() -> {
//                    if (!robot.hasSample()) while (!robot.deposit.chamberReady()) robot.deposit.nextState();
                }),
                new SleepAction(WAIT_APPROACH_CHAMBER),
//                telemetryPacket -> !(robot.deposit.arm.atPosition(Arm.SPECIMEN) && robot.deposit.lift.atPosition(HEIGHT_CHAMBER_HIGH)), // wait until deposit in position
//                new InstantAction(robot.deposit::nextState),
//                telemetryPacket -> robot.deposit.hasSample(), // wait until spec scored
                new SleepAction(WAIT_SCORE_CHAMBER)
        );
    }

    private static Action sweep() {
        return new SequentialAction(
//                new InstantAction(() -> robot.intake.sweeper.setActivated(true)),
                new SleepAction(WAIT_SWEEPER_EXTEND),
//                new InstantAction(() -> robot.intake.sweeper.setActivated(false)),
                new SleepAction(WAIT_SWEEPER_RETRACT)
        );
    }

    private static Action finishIntaking() {
        return new SequentialAction(
//                new InstantAction(() -> robot.intake.runRoller(1)),
                new SleepAction(WAIT_POST_INTAKING)
//                new InstantAction(() -> robot.intake.runRoller(0))
        );
    }

    private static Action approachSub() {
        return new SequentialAction(
//                new InstantAction(() -> robot.intake.sweeper.setActivated(true)),
                new SleepAction(WAIT_SWEEPER_EXTEND),
//                new InstantAction(() -> robot.intake.runRoller(SPEED_INTAKING)),
                new SleepAction(WAIT_DROP_TO_EXTEND),
//                new InstantAction(() -> robot.intake.extendo.runManual(SPEED_EXTEND)),
                new SleepAction(TIME_EXTEND)
//                new InstantAction(() -> robot.intake.sweeper.setActivated(false))
        );
    }

    private static Action preExtend() {
        return new SequentialAction(
//                telemetryPacket -> !robot.deposit.hasSample(),
//                new SleepAction(Deposit.TIME_TRANSFERRING + TIME_EXITING_BUCKET + 0.5 * TIME_TO_BASKET),
                new InstantAction(() -> {
//                    robot.intake.extendo.setTarget(EXTEND_APPROACH_SPIKES);
//                    robot.intake.runRoller(SPEED_INTAKING);
                })
        );
    }
}