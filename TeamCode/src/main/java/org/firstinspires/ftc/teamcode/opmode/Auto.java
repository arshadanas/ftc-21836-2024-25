package org.firstinspires.ftc.teamcode.opmode;

import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.A;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_DOWN;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_UP;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.X;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.Y;
import static org.firstinspires.ftc.teamcode.control.vision.AutoSampleAligner.LL_TURN_MULTIPLIER;
import static org.firstinspires.ftc.teamcode.control.vision.pipeline.Sample.NEUTRAL;
import static org.firstinspires.ftc.teamcode.opmode.Auto.AutonConfig.CONFIRMING;
import static org.firstinspires.ftc.teamcode.opmode.Auto.AutonConfig.EDITING_ALLIANCE;
import static org.firstinspires.ftc.teamcode.opmode.Auto.AutonConfig.EDITING_CYCLES;
import static org.firstinspires.ftc.teamcode.opmode.Auto.AutonConfig.EDITING_SIDE;
import static org.firstinspires.ftc.teamcode.opmode.Auto.State.DRIVING_TO_SUB;
import static org.firstinspires.ftc.teamcode.opmode.Auto.State.INTAKING_1;
import static org.firstinspires.ftc.teamcode.opmode.Auto.State.INTAKING_2;
import static org.firstinspires.ftc.teamcode.opmode.Auto.State.INTAKING_3;
import static org.firstinspires.ftc.teamcode.opmode.Auto.State.PARKING;
import static org.firstinspires.ftc.teamcode.opmode.Auto.State.SCORING_PRELOAD;
import static org.firstinspires.ftc.teamcode.opmode.Auto.State.SCORING;
import static org.firstinspires.ftc.teamcode.opmode.Auto.State.SCORING_1;
import static org.firstinspires.ftc.teamcode.opmode.Auto.State.SCORING_2;
import static org.firstinspires.ftc.teamcode.opmode.Auto.State.TAKING_PICTURE;
import static org.firstinspires.ftc.teamcode.opmode.Auto.State.SUB_INTAKING;
import static org.firstinspires.ftc.teamcode.subsystem.Deposit.HEIGHT_BASKET_HIGH;
import static org.firstinspires.ftc.teamcode.subsystem.Deposit.HEIGHT_CHAMBER_HIGH;
import static java.lang.Math.PI;
import static java.lang.Math.abs;
import static java.lang.Math.hypot;
import static java.lang.Math.min;
import static java.lang.Math.toRadians;
import static java.lang.Math.ceil;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.TurnConstraints;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.control.FirstTerminateAction;
import org.firstinspires.ftc.teamcode.control.motion.EditablePose;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.control.vision.AutoSampleAligner;
import org.firstinspires.ftc.teamcode.subsystem.Deposit;
import org.firstinspires.ftc.teamcode.control.vision.LimelightEx;
import org.firstinspires.ftc.teamcode.subsystem.Robot;
import org.firstinspires.ftc.teamcode.subsystem.utility.cachedhardware.CachedDcMotorEx;

import java.util.Arrays;

@Config
@Autonomous(preselectTeleOp = "Tele")
public final class Auto extends LinearOpMode {

    enum State {
        SCORING_PRELOAD,
        INTAKING_PARTNER_SAMPLE,
        SCORING_PARTNER_SAMPLE,
        INTAKING_1,
        SCORING_1,
        INTAKING_2,
        SCORING_2,
        INTAKING_3,
        SCORING,
        DRIVING_TO_SUB,
        TAKING_PICTURE,
        SUB_INTAKING,
        PARKING
    }

    public static Telemetry mTelemetry;

    public static void divider() {
        mTelemetry.addLine();
        mTelemetry.addLine("--------------------------------------------------------------------------");
        mTelemetry.addLine();
    }

    public static double
            DEAD_TIME = 0,
            LENGTH_ROBOT = 14.386976771653545,
            WIDTH_ROBOT = 14.28740157480315,
            SIZE_HALF_FIELD = 70.5,
            SIZE_TILE = 23.625,
            DISTANCE_BETWEEN_SPECIMENS = 2,

            LL_ANGLE_BUCKET_INCREMENT = 50,
            LL_DISTANCE_START_LOWERING = 13,
            LL_EXTEND_OFFSET = -9.5,
            LL_MAX_PICTURE_TIME = 3,
            LL_MIN_PICTURE_TIME = 0,
            LL_NO_DETECTION_Y_MOVE = 3,
            LL_SPEED_MAX_EXTENDO = 1,
            LL_SWEEP_ANGLE_RANGE = 10,
            LL_SWEEP_SPEED = 0.5,
            LL_WAIT_INTAKE = 1,

            WAIT_MAX_INTAKE = 1,

            ANGLE_PITCH_SPIKES = 0.5,
            ANGLE_PITCH_FROM_SUB = 0.5,

            EXTEND_SAMPLE_1 = 21,
            EXTEND_SAMPLE_2 = 20,
            EXTEND_SAMPLE_3 = 24,

            PRE_EXTEND_SAMPLE_1 = 12.5,
            PRE_EXTEND_SAMPLE_2 = 12,
            PRE_EXTEND_SAMPLE_3 = 12,

            VEL_INCHING = 5,
            VEL_ANG_INCHING = 0.75,
            VEL_ANG_INTAKING_3 = 2,

            WAIT_MAX_BEFORE_RE_SEARCH = 1,
            WAIT_BEFORE_SCORING_PRELOAD = 0.8,
            WAIT_APPROACH_WALL = 0,
            WAIT_APPROACH_BASKET = 0,
            WAIT_APPROACH_CHAMBER = 0,
            WAIT_SCORE_BASKET = 0.2,
            WAIT_SCORE_CHAMBER = 0.1,
            WAIT_INTAKE_RETRACT_POST_SUB = 0,
            WAIT_EXTEND_MAX_SPIKE = 2,

            LIFT_HEIGHT_TOLERANCE = 3.75,

            X_OFFSET_CHAMBER_1 = 1,
            X_OFFSET_CHAMBER_2 = -1,
            X_OFFSET_CHAMBER_3 = -2,
            X_OFFSET_CHAMBER_4 = -3,
            X_OFFSET_CHAMBER_5 = -4,

            SPEED_DELIVERY = -0.75,
            TIME_DELIVERY = 0.5,

            WAIT_BEFORE_EXTENDING_FROM_SUB = 1,
            WAIT_EXTEND_MAX_SPEC_INTAKE = 2,
            WAIT_EXTEND_MAX_DELIVERY = 2,
            WAIT_AFTER_INTAKING_SPEC_3_TO_DELIVERY = 1,

            EXTEND_SUB_DELIVERY = 20,
            PRE_EXTEND_SPEC_1 = 12,
            EXTEND_SPEC_1 = 20,
            PRE_EXTEND_SPEC_2 = 12,
            EXTEND_SPEC_2 = 20,
            PRE_EXTEND_SPEC_3 = 12,
            EXTEND_SPEC_3 = 20,

            Y_INCHING_FORWARD_WHEN_INTAKING = 10,

            TIME_CYCLE = 5.5,
            TIME_PARK = 2.5;

    /// <a href="https:///www.desmos.com/calculator/l8pl2gf1mb">Adjust spikes 1 and 2</a>
    /// <a href="https://www.desmos.com/calculator/sishohvpwc">Visualize spike samples</a>
    public static EditablePose
            admissibleError = new EditablePose(1, 1, 0.05),
            admissibleVel = new EditablePose(25, 25, toRadians(30)),

            basketError = new EditablePose(1, 1, toRadians(4)),
            basketVelError = new EditablePose(12, 12, toRadians(30)),

            intaking1 = new EditablePose(-61, -54, PI/3),
            intaking2 = new EditablePose(-62, -51.5, 1.4632986527692424),
            intaking3 = new EditablePose(-58, -50, 2 * PI / 3),

            snapshotPos = new EditablePose(-25, -10, toRadians(20)),

            scoring = new EditablePose(-56, -56, PI / 4),
            scoringFromSub = new EditablePose(-57.25, -57.25, PI/4),

            sample1 = new EditablePose(-48, -26.8, PI / 2),
            sample2 = new EditablePose(-60, -27.4, PI / 2),
            sample3 = new EditablePose(-69, -27.8, PI / 2),

            park1 = new EditablePose(-0.5 * SIZE_TILE, -2 * SIZE_TILE, PI),
            park2 = new EditablePose(-1.5 * SIZE_TILE, -2 * SIZE_TILE, PI / 2),
            park3 = new EditablePose(-2.5 * SIZE_TILE, -2 * SIZE_TILE, PI / 2),

            sub = new EditablePose(-22.5, -11, 0),

            aroundBeamPushing = new EditablePose(35, -30, PI / 2),

            chamberRight = new EditablePose(0.5 * WIDTH_ROBOT + 0.375, -33, PI / 2),

            specSpike1 = new EditablePose(48, -26.8, PI / 2),
            specSpike2 = new EditablePose(60, -27.4, PI / 2),
            specSpike3 = new EditablePose(69, -27.8, PI / 2),

            specIntaking1 = new EditablePose(23, -48, toRadians(-80)),
            specIntaking2 = new EditablePose(24, -48, toRadians(-70)),
            specIntaking3 = new EditablePose(25, -48, - PI / 2),

            deliveryPos = new EditablePose(36, -50, 0),

            intakingSpec = new EditablePose(36, -60, PI / 2),

            targetOffset = new EditablePose(0 ,0, 0);

    static Pose2d pose = new Pose2d(0,0, 0.5 * PI);
    static boolean isRedAlliance = false;

    enum AutonConfig {
        CONFIRMING,
        EDITING_ALLIANCE,
        EDITING_SIDE,
        EDITING_CYCLES;

        public static final AutonConfig[] selections = values();

        public AutonConfig plus(int i) {
            int max = selections.length;
            return selections[((ordinal() + i) % max + max) % max];
        }
        public String markIf(AutonConfig s) {
            return this == s ? " <" : "";
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {

        intaking1.heading = intaking1.angleTo(sample1);
        intaking2.heading = intaking2.angleTo(sample2);
        intaking3.heading = intaking3.angleTo(sample3);

        specIntaking1.heading = specIntaking1.angleTo(specSpike1);
        specIntaking2.heading = specIntaking2.angleTo(specSpike2);
        specIntaking3.heading = specIntaking3.angleTo(specSpike3);

        // Initialize multiple telemetry outputs:
        mTelemetry = new MultipleTelemetry(telemetry);

        // Initialize robot:
        Robot robot = new Robot(hardwareMap, pose);
//        robot.deposit.claw.turnToAngle(ANGLE_CLAW_SAMPLE);

        // Initialize gamepads:
        GamepadEx gamepadEx1 = new GamepadEx(gamepad1);

        AutonConfig selection = EDITING_ALLIANCE;

        boolean specimenSide = false;
        int cycles = 5;

        ElapsedTime timer = new ElapsedTime();

        config:
        while (opModeInInit() && timer.seconds() < 5) {
            gamepadEx1.readButtons();

            boolean up = gamepadEx1.wasJustPressed(DPAD_UP);
            boolean down = gamepadEx1.wasJustPressed(DPAD_DOWN);
            boolean y = gamepadEx1.wasJustPressed(Y);
            boolean x = gamepadEx1.wasJustPressed(X);
            boolean a = gamepadEx1.wasJustPressed(A);

            if (up || down || y || a || x) timer.reset();

            if (up)
                do selection = selection.plus(-1);
                while (selection == EDITING_CYCLES && !specimenSide);
            else if (down)
                do selection = selection.plus(1);
                while (selection == EDITING_CYCLES && !specimenSide);

            switch (selection) {
                case CONFIRMING:
                    if (x) break config;
                case EDITING_ALLIANCE:
                    if (x) isRedAlliance = !isRedAlliance;
                    break;
                case EDITING_SIDE:
                    if (x) specimenSide = !specimenSide;
                    break;
                case EDITING_CYCLES:
                    if (specimenSide && y) cycles++;
                    if (specimenSide && a && cycles > 0) cycles--;
                    break;
            }

            printConfig(false, timer.seconds(), selection, specimenSide, cycles);
        }

        Limelight3A limelight3a = hardwareMap.get(Limelight3A.class, "limelight");
        AutoSampleAligner sampleAligner = new AutoSampleAligner(new LimelightEx(limelight3a, hardwareMap));
        sampleAligner.activateLimelight(
                specimenSide ?
                /*specimen*/ isRedAlliance ? AutoSampleAligner.Pipeline.RED : AutoSampleAligner.Pipeline.BLUE :
                /*sample*/   isRedAlliance ? AutoSampleAligner.Pipeline.YELLOW_RED : AutoSampleAligner.Pipeline.YELLOW_BLUE
        );
        limelight3a.stop();
        limelight3a.start();

        robot.intake.setAlliance(isRedAlliance);

        Action trajectory;

        CachedDcMotorEx[] dtMotors = {
                robot.drivetrain.leftFront,
                robot.drivetrain.leftBack,
                robot.drivetrain.rightBack,
                robot.drivetrain.rightFront,
        };

        if (specimenSide) {

            robot.intake.specimenMode = true;

            robot.deposit.preloadSpecimen();

            pose = new Pose2d(chamberRight.x, 0.5 * LENGTH_ROBOT - SIZE_HALF_FIELD, PI / 2);

            Action scorePreload = robot.drivetrain.actionBuilder(pose)
                    .strafeTo(chamberRight.toVector2d())
                    .stopAndAdd(scoreSpecimen(robot))
                    .build();

            Action deliverSub = robot.drivetrain.actionBuilder(chamberRight.toPose2d())
                    .afterTime(WAIT_BEFORE_EXTENDING_FROM_SUB, () -> {
                        robot.intake.extendo.setTarget(EXTEND_SUB_DELIVERY);
                        robot.intake.setRollerAndAngle(1);
                        robot.intake.doTransfer = false;
                        robot.intake.retractBucketBeforeExtendo = true;
                    })
                    .strafeToLinearHeading(specIntaking1.toVector2d(), specIntaking1.angleTo(deliveryPos))
                    .stopAndAdd(new FirstTerminateAction(
                            t -> !robot.intake.extendo.atPosition(EXTEND_SUB_DELIVERY),
                            new SleepAction(WAIT_EXTEND_MAX_DELIVERY)
                    ))
                    .stopAndAdd(() -> robot.intake.setRoller(SPEED_DELIVERY))
                    .waitSeconds(TIME_DELIVERY)
                    .stopAndAdd(() -> robot.intake.setRollerAndAngle(1))
                    .build();

            Action intake1 = robot.drivetrain.actionBuilder(new Pose2d(specIntaking1.toVector2d(), specIntaking1.angleTo(deliveryPos)))
                    .stopAndAdd(() -> robot.intake.extendo.setTarget(PRE_EXTEND_SPEC_1))
                    .turnTo(specIntaking1.heading)
                    .stopAndAdd(() -> {
                        robot.intake.extendo.setTarget(EXTEND_SPEC_1);
                    })
                    .stopAndAdd(new FirstTerminateAction(
                            t -> !(robot.intake.hasSample() || robot.intake.extendo.atPosition(EXTEND_SPEC_1)),
                            new SleepAction(WAIT_EXTEND_MAX_SPEC_INTAKE)
                    ))
                    .waitSeconds(WAIT_MAX_INTAKE)
                    .build();

            Action deliver1 = robot.drivetrain.actionBuilder(specIntaking1.toPose2d())
                    .stopAndAdd(() -> {
                        robot.intake.extendo.setTarget(PRE_EXTEND_SPEC_2);
                    })
                    .strafeToLinearHeading(specIntaking2.toVector2d(), specIntaking2.angleTo(deliveryPos))
                    .stopAndAdd(new FirstTerminateAction(
                            t -> !robot.intake.extendo.atPosition(PRE_EXTEND_SPEC_2),
                            new SleepAction(WAIT_EXTEND_MAX_DELIVERY)
                    ))
                    .stopAndAdd(() -> robot.intake.setRoller(SPEED_DELIVERY))
                    .waitSeconds(TIME_DELIVERY)
                    .stopAndAdd(() -> robot.intake.setRollerAndAngle(1))
                    .build();

            Action intake2 = robot.drivetrain.actionBuilder(new Pose2d(specIntaking2.toVector2d(), specIntaking2.angleTo(deliveryPos)))
                    .stopAndAdd(() -> robot.intake.extendo.setTarget(PRE_EXTEND_SPEC_2))
                    .turnTo(specIntaking2.heading)
                    .stopAndAdd(() -> robot.intake.extendo.setTarget(EXTEND_SPEC_2))
                    .stopAndAdd(new FirstTerminateAction(
                            t -> !(robot.intake.hasSample() || robot.intake.extendo.atPosition(EXTEND_SPEC_2)),
                            new SleepAction(WAIT_EXTEND_MAX_SPEC_INTAKE)
                    ))
                    .waitSeconds(WAIT_MAX_INTAKE)
                    .build();

            Action deliver2 = robot.drivetrain.actionBuilder(specIntaking2.toPose2d())
                    .stopAndAdd(() -> {
                        robot.intake.extendo.setTarget(PRE_EXTEND_SPEC_3);
                    })
                    .strafeToLinearHeading(specIntaking3.toVector2d(), specIntaking3.angleTo(deliveryPos))
                    .stopAndAdd(new FirstTerminateAction(
                            t -> !robot.intake.extendo.atPosition(PRE_EXTEND_SPEC_3),
                            new SleepAction(WAIT_EXTEND_MAX_DELIVERY)
                    ))
                    .stopAndAdd(() -> robot.intake.setRoller(SPEED_DELIVERY))
                    .waitSeconds(TIME_DELIVERY)
                    .stopAndAdd(() -> robot.intake.setRollerAndAngle(1))
                    .build();

            Action intake3 = robot.drivetrain.actionBuilder(new Pose2d(specIntaking3.toVector2d(), specIntaking3.angleTo(deliveryPos)))
                    .stopAndAdd(() -> robot.intake.extendo.setTarget(PRE_EXTEND_SPEC_3))
                    .turnTo(specIntaking3.heading)
                    .stopAndAdd(() -> {
                        robot.intake.extendo.setTarget(EXTEND_SPEC_3);
                        robot.deposit.nextState();
                    })
                    .stopAndAdd(new FirstTerminateAction(
                            t -> !(robot.intake.hasSample() || robot.intake.extendo.atPosition(EXTEND_SPEC_3)),
                            new SleepAction(WAIT_EXTEND_MAX_SPEC_INTAKE)
                    ))
                    .waitSeconds(WAIT_MAX_INTAKE)
                    .build();

            TrajectoryActionBuilder builder = robot.drivetrain.actionBuilder(specIntaking2.toPose2d())
                    .stopAndAdd(() -> {
                        robot.intake.extendo.setTarget(PRE_EXTEND_SPEC_3);
                    })
                    .turnTo(specIntaking3.angleTo(deliveryPos))
                    .stopAndAdd(new FirstTerminateAction(
                            t -> !robot.intake.extendo.atPosition(PRE_EXTEND_SPEC_3),
                            new SleepAction(WAIT_EXTEND_MAX_DELIVERY)
                    ))
                    .stopAndAdd(() -> robot.intake.setRoller(SPEED_DELIVERY))
                    .waitSeconds(TIME_DELIVERY)
                    .stopAndAdd(() -> robot.intake.setRollerAndAngle(0));

            /// Push samples
            builder = builder
                    .setTangent(-PI / 2);

            double[] chamberXs = {
                    X_OFFSET_CHAMBER_1,
                    X_OFFSET_CHAMBER_2,
                    X_OFFSET_CHAMBER_3,
                    X_OFFSET_CHAMBER_4,
                    X_OFFSET_CHAMBER_5,
            };

            /// Cycle specimens
            for (int i = 0; i < min(chamberXs.length, cycles); i++) {
                if (i > 0) builder = builder
                        .afterTime(0, new SequentialAction(
                                t -> robot.deposit.state != Deposit.State.STANDBY,
                                new InstantAction(robot.deposit::nextState)
                        ))
                        .setTangent(- PI / 2)
                        .splineToConstantHeading(intakingSpec.toVector2d(), - PI / 2)
                ;
                builder = builder
                        .waitSeconds(WAIT_APPROACH_WALL)
                        .afterTime(0, robot.deposit::nextState)
                        .stopAndAdd(telemetryPacket -> !robot.deposit.specGrabbed())
                        .setTangent(PI / 2)
                        .splineToConstantHeading(new Vector2d(chamberRight.x + chamberXs[i] * DISTANCE_BETWEEN_SPECIMENS, chamberRight.y), PI / 2)
                        .stopAndAdd(scoreSpecimen(robot))
                ;
            }

            /// Park in observation zone
            builder = builder.strafeTo(intakingSpec.toVector2d());

            Action scoreAll = builder.build();

            trajectory = new Action() {

                State state = SCORING_PRELOAD;

                Action snapshotAction = null;

                ElapsedTime matchTimer = null;

                Action activeTraj = scorePreload;

                int subCycle = 1;

                void stopDt() {
                    for (CachedDcMotorEx motor : dtMotors) motor.setPower(0);
                }

                public boolean run(@NonNull TelemetryPacket p) {
                    if (matchTimer == null) matchTimer = new ElapsedTime();

                    double remaining = (30 - DEAD_TIME) - matchTimer.seconds();

                    boolean trajDone = !activeTraj.run(p);

                    switch (state) {
                        case SCORING_PRELOAD:
                            robot.headlight.setActivated(true);
                            if (trajDone) {
                                state = TAKING_PICTURE;
                                timer.reset();
                                snapshotAction = new SequentialAction(
                                        new SleepAction(LL_MIN_PICTURE_TIME),
                                        new FirstTerminateAction(
                                                sampleAligner.detectTarget(),
                                                new SleepAction(LL_MAX_PICTURE_TIME)
                                        )
                                );
                            }
                            break;
                        case TAKING_PICTURE:

                            boolean done = snapshotAction.run(p);

                            EditablePose offset = new EditablePose(sampleAligner.getTargetOffset());

                            if (!(offset.x == 0 && offset.y == 0 && offset.heading == 0)) {

                                targetOffset = offset;
                                double extendoInches = hypot(targetOffset.x, targetOffset.y) + LL_EXTEND_OFFSET;

                                robot.intake.extendo.powerCap = LL_SPEED_MAX_EXTENDO;

                                activeTraj = robot.drivetrain.actionBuilder(robot.drivetrain.pose)
                                        .turn(-targetOffset.heading * LL_TURN_MULTIPLIER)
                                        .stopAndAdd(() -> {
                                            robot.intake.extendo.setTarget(extendoInches);
                                            robot.intake.setAngle(0.01);
                                            robot.intake.setRoller(0);
                                        })
                                        .stopAndAdd(new FirstTerminateAction(
                                                t -> robot.intake.extendo.getPosition() < extendoInches - LL_DISTANCE_START_LOWERING,
                                                new SleepAction(1)
                                        ))
                                        .stopAndAdd(() -> robot.intake.setRoller(1))
                                        .stopAndAdd(timer::reset)
                                        .afterTime(0, t -> !robot.intake.setAngle(timer.seconds() * LL_ANGLE_BUCKET_INCREMENT))
                                        .waitSeconds(LL_WAIT_INTAKE)
                                        .build();

                                state = SUB_INTAKING;

                            } else if (done) searchAgainForSample(robot);

                            break;

                        case SUB_INTAKING:

                            if (robot.hasSample()) {
                                robot.headlight.setActivated(false);
                                Pose2d current = robot.drivetrain.pose;

                                robot.intake.extendo.powerCap = 1;

                                activeTraj = deliverSub;

                                subCycle++;
                                state = DRIVING_TO_SUB;
                                stopDt();

                            } else if (trajDone) searchAgainForSample(robot);

                            break;

                        case DRIVING_TO_SUB:

                            if (trajDone) {
                                activeTraj = intake1;
                                state = INTAKING_1;
                            }
                            break;

                        case INTAKING_1:

                            // Sample intaked
                            if (robot.intake.hasSample()) {
                                activeTraj = deliver1;
                                state = SCORING_1;
                                stopDt();
                            }
                            else if (trajDone) { // skip to 2 if didn't get 1
                                activeTraj = robot.drivetrain.actionBuilder(specIntaking1.toPose2d())
                                        .stopAndAdd(() -> robot.intake.extendo.setTarget(PRE_EXTEND_SPEC_2))
                                        .strafeToLinearHeading(specIntaking2.toVector2d(), specIntaking2.heading)
                                        .stopAndAdd(() -> robot.intake.extendo.setTarget(EXTEND_SPEC_2))
                                        .stopAndAdd(new FirstTerminateAction(
                                                t -> !(robot.intake.hasSample() || robot.intake.extendo.atPosition(EXTEND_SPEC_2)),
                                                new SleepAction(WAIT_EXTEND_MAX_SPEC_INTAKE)
                                        ))
                                        .waitSeconds(WAIT_MAX_INTAKE)
                                        .build();
                                state = INTAKING_2;
                                robot.intake.extendo.setExtended(false);
                                robot.intake.ejectSample();
                            }

                            break;

                        case SCORING_1:

                            if (trajDone) {
                                activeTraj = intake2;
                                state = INTAKING_2;
                            }
                            break;

                        case INTAKING_2:

                            // Sample intaked
                            if (robot.intake.hasSample()) {
                                activeTraj = deliver2;
                                state = SCORING_2;
                                stopDt();
                            }
                            else if (trajDone) { // skip to 3 if didn't get 2
                                activeTraj = robot.drivetrain.actionBuilder(specIntaking2.toPose2d())
                                        .stopAndAdd(() -> robot.intake.extendo.setTarget(PRE_EXTEND_SPEC_3))
                                        .strafeToLinearHeading(specIntaking3.toVector2d(), specIntaking3.heading)
                                        .stopAndAdd(() -> robot.intake.extendo.setTarget(EXTEND_SPEC_3))
                                        .stopAndAdd(new FirstTerminateAction(
                                                t -> !(robot.intake.hasSample() || robot.intake.extendo.atPosition(EXTEND_SPEC_3)),
                                                new SleepAction(WAIT_EXTEND_MAX_SPEC_INTAKE)
                                        ))
                                        .waitSeconds(WAIT_MAX_INTAKE)
                                        .build();
                                state = INTAKING_3;
                                robot.intake.extendo.setExtended(false);
                                robot.intake.ejectSample();
                            }

                            break;

                        case SCORING_2:

                            if (trajDone) {
                                activeTraj = intake3;
                                state = INTAKING_3;
                            }
                            break;

                        case INTAKING_3:

                            // Sample intaked
                            if (robot.intake.hasSample() || trajDone) {
                                activeTraj = scoreAll;
                                state = SCORING;
                                stopDt();
                            }

                            break;

                        case SCORING:
                            return !trajDone;
                    }

                    return true;
                }

                private void searchAgainForSample(Robot robot) {
                    robot.intake.setRollerAndAngle(0);
                    robot.intake.extendo.setExtended(false);
                    robot.intake.ejectSample();

                    activeTraj = new SleepAction(WAIT_MAX_BEFORE_RE_SEARCH);

                    state = SCORING_PRELOAD;
                }
            };

        } else {

            robot.deposit.preloadSample();

            robot.intake.retractBucketBeforeExtendo = false;
            robot.deposit.requireDistBeforeLoweringLift = false;

            pose = new Pose2d(0.5 * LENGTH_ROBOT + 0.375 - 2 * SIZE_TILE, 0.5 * WIDTH_ROBOT - SIZE_HALF_FIELD, 0);

            TurnConstraints llSweepConstraint = new TurnConstraints(LL_SWEEP_SPEED, -MecanumDrive.PARAMS.maxAngAccel, MecanumDrive.PARAMS.maxAngAccel);

            MinVelConstraint inchingConstraint = new MinVelConstraint(Arrays.asList(
                    new TranslationalVelConstraint(VEL_INCHING),
                    new AngularVelConstraint(VEL_ANG_INCHING)
            ));

            robot.deposit.setWristPitchingAngle(ANGLE_PITCH_SPIKES);

            // wait until deposit in position
            Action scorePreload = robot.drivetrain.actionBuilder(pose)
                    .afterTime(0, preExtend(robot, PRE_EXTEND_SAMPLE_1))
                    .afterTime(WAIT_BEFORE_SCORING_PRELOAD, scoreSample(robot))
                    .strafeToLinearHeading(intaking1.toVector2d(), intaking1.heading)
                    .build();

            Action intake1 = new SequentialAction(
                    new InstantAction(() -> {
                        robot.intake.setRollerAndAngle(1);
                        robot.intake.extendo.setTarget(EXTEND_SAMPLE_1);
                    }),
                    new FirstTerminateAction(
                            t -> !robot.intake.extendo.atPosition(EXTEND_SAMPLE_1),
                            t -> !robot.intake.hasSample(),
                            new SleepAction(WAIT_EXTEND_MAX_SPIKE)
                    ),
                    new SleepAction(WAIT_MAX_INTAKE)
            );

            Action score1 = robot.drivetrain.actionBuilder(intaking1.toPose2d())
                    .afterTime(0, preExtend(robot, PRE_EXTEND_SAMPLE_2))
                    .strafeToLinearHeading(intaking2.toVector2d(), intaking2.heading)
                    .stopAndAdd(scoreSample(robot))
                    .build();

            Action intake2 = new SequentialAction(
                    new InstantAction(() -> {
                        robot.intake.setRollerAndAngle(1);
                        robot.intake.extendo.setTarget(EXTEND_SAMPLE_2);
                    }),
                    new FirstTerminateAction(
                            t -> !robot.intake.extendo.atPosition(EXTEND_SAMPLE_2),
                            t -> !robot.intake.hasSample(),
                            new SleepAction(WAIT_EXTEND_MAX_SPIKE)
                    ),
                    new SleepAction(WAIT_MAX_INTAKE)
            );

            Action score2 = new ParallelAction(
                    preExtend(robot, PRE_EXTEND_SAMPLE_3),
                    scoreSample(robot)
            );

            Action intake3 = robot.drivetrain.actionBuilder(intaking2.toPose2d())
                    .strafeToLinearHeading(intaking3.toVector2d(), intaking3.heading, new AngularVelConstraint(VEL_ANG_INTAKING_3))
                    .stopAndAdd(() -> {
                        robot.intake.setRollerAndAngle(1);
                        robot.intake.extendo.setTarget(EXTEND_SAMPLE_3);
                    })
                    .stopAndAdd(new FirstTerminateAction(
                            t -> !robot.intake.extendo.atPosition(EXTEND_SAMPLE_3),
                            t -> !robot.intake.hasSample(),
                            new SleepAction(WAIT_EXTEND_MAX_SPIKE)
                    ))
                    .setTangent(PI / 2)
                    .lineToY(intaking3.y + Y_INCHING_FORWARD_WHEN_INTAKING, inchingConstraint)
                    .build();

            Action score3 = robot.drivetrain.actionBuilder(intaking3.toPose2d())
                    .stopAndAdd(new InstantAction(() -> robot.intake.setRollerAndAngle(0)))
                    .strafeToLinearHeading(intaking2.toVector2d(), intaking2.heading)
                    .stopAndAdd(scoreSample(robot))
                    .build();

            Action park = robot.drivetrain.actionBuilder(scoring.toPose2d())
                    .afterTime(0, () -> robot.deposit.lift.setTarget(0))
                    .splineTo(sub.toVector2d(), sub.heading)
                    .build();

            trajectory = new Action() {

                State state = SCORING_PRELOAD;

                Action snapshotAction = null;

                ElapsedTime matchTimer = null;

                Action activeTraj = scorePreload;

                int subCycle = 1, barnacle = 0;

                void stopDt() {
                    for (CachedDcMotorEx motor : dtMotors) motor.setPower(0);
                }

                public boolean run(@NonNull TelemetryPacket p) {
                    if (matchTimer == null) matchTimer = new ElapsedTime();

                    double remaining = (30 - DEAD_TIME) - matchTimer.seconds();

                    boolean trajDone = !activeTraj.run(p);

                    switch (state) {
                        case SCORING_PRELOAD:

                            if (trajDone
//                                    || atPose(robot, intaking1) && !robot.hasSample()
                            ) {
                                stopDt();
                                activeTraj = intake1;
                                state = INTAKING_1;
                            }
                            break;

                        case INTAKING_1:

                            // Sample intaked
                            if (robot.intake.hasSample()) {
                                robot.intake.setRollerAndAngle(0);
                                activeTraj = score1;
                                state = SCORING_1;
                                stopDt();
                            }
                            else if (trajDone || barnacle == 1) { // skip to 2 if didn't get 1
                                activeTraj = robot.drivetrain.actionBuilder(robot.drivetrain.pose)
                                        .afterTime(0, () -> robot.intake.extendo.setExtended(false))
                                        .strafeToLinearHeading(intaking2.toVector2d(), intaking2.heading)
                                        .stopAndAdd(() -> {
                                            robot.intake.setRollerAndAngle(1);
                                            robot.intake.extendo.setTarget(EXTEND_SAMPLE_2);
                                        })
                                        .stopAndAdd(new FirstTerminateAction(
                                                t -> !robot.intake.extendo.atPosition(EXTEND_SAMPLE_2),
                                                t -> !robot.intake.hasSample(),
                                                new SleepAction(WAIT_EXTEND_MAX_SPIKE)
                                        ))
                                        .waitSeconds(WAIT_MAX_INTAKE)
//                                        .setTangent(scoring1Intaking2Scoring2.heading)
//                                        .lineToY(scoring1Intaking2Scoring2.y + Y_INCHING_FORWARD_WHEN_INTAKING, inchingConstraint)
                                        .build();
                                state = INTAKING_2;
                                robot.intake.extendo.setExtended(false);
                                robot.intake.ejectSample();
                            }

                            break;

                        case SCORING_1:
                            if (trajDone) {
                                if (barnacle == 2) {
                                    activeTraj = intake3;
                                    state = INTAKING_3;
                                } else {
                                    activeTraj = intake2;
                                    state = INTAKING_2;
                                }
                            }
                            break;
                        case INTAKING_2:

                            // Sample intaked
                            if (robot.intake.hasSample()) {
                                robot.intake.setRollerAndAngle(0);
                                activeTraj = score2;
                                state = SCORING_2;
                                stopDt();
                            }
                            else if (trajDone) { // skip to 3 if didn't get 2
                                activeTraj = robot.drivetrain.actionBuilder(intaking2.toPose2d())
                                        .strafeToLinearHeading(intaking3.toVector2d(), intaking3.heading)
                                        .stopAndAdd(() -> {
                                            robot.intake.setRollerAndAngle(1);
                                            robot.intake.extendo.setTarget(EXTEND_SAMPLE_3);
                                        })
                                        .stopAndAdd(new FirstTerminateAction(
                                                t -> !robot.intake.extendo.atPosition(EXTEND_SAMPLE_3),
                                                t -> !robot.intake.hasSample(),
                                                new SleepAction(WAIT_EXTEND_MAX_SPIKE)
                                        ))
                                        .setTangent(intaking3.heading)
                                        .lineToY(intaking3.y + Y_INCHING_FORWARD_WHEN_INTAKING, inchingConstraint)
                                        .build();
                                state = INTAKING_3;
                                robot.intake.extendo.setExtended(false);
                                robot.intake.ejectSample();
                            }

                            break;

                        case SCORING_2:
                            if (trajDone) {

                                if (barnacle == 3) {
                                    Pose2d current = robot.drivetrain.pose;
                                    activeTraj = robot.drivetrain.actionBuilder(current)
                                            .setTangent(current.heading.toDouble())
                                            .splineTo(snapshotPos.toVector2d(), snapshotPos.heading)
                                            .build();
                                    state = DRIVING_TO_SUB;
                                } else {
                                    activeTraj = intake3;
                                    state = INTAKING_3;
                                }
                            }
                            break;
                        case INTAKING_3:

                            // Sample intaked
                            if (robot.intake.hasSample()) {
                                robot.intake.setRollerAndAngle(0);
                                activeTraj = score3;
                                state = SCORING;
                                stopDt();
                            }
                            else if (trajDone) { // skip to sub if didn't get 3
                                activeTraj = robot.drivetrain.actionBuilder(intaking3.toPose2d())
                                        .setTangent(scoring.heading)
                                        .splineToSplineHeading(snapshotPos.toPose2d(), snapshotPos.heading)
                                        .build();
                                state = DRIVING_TO_SUB;
                                robot.intake.setRollerAndAngle(0);
                                robot.intake.ejectSample();
                                robot.intake.extendo.setExtended(false);
                            }

                            break;

                        case SCORING:
                            if (trajDone) {
                                Pose2d current = robot.drivetrain.pose;
                                if (remaining < TIME_CYCLE) {

                                    activeTraj = barnacle == 1 ?
                                            robot.drivetrain.actionBuilder(current)
                                                            .setTangent(current.heading)
                                                            .splineTo(park1.toVector2d(), 0)
                                                            .build():
                                            robot.drivetrain.actionBuilder(current)
                                                    .strafeToLinearHeading((barnacle == 2 ? park2 : park3).toVector2d(), park2.heading)
                                                    .build();
                                    state = PARKING;
                                    stopDt();
                                } else {
                                    activeTraj = robot.drivetrain.actionBuilder(current)
                                            .setTangent(current.heading.toDouble())
                                            .splineTo(snapshotPos.toVector2d(), snapshotPos.heading)
                                            .build();
                                    state = DRIVING_TO_SUB;
                                }
                            }
                            break;

                        case DRIVING_TO_SUB:
                            robot.headlight.setActivated(true);
                            if (trajDone) {
                                state = TAKING_PICTURE;
                                timer.reset();
                                snapshotAction = new SequentialAction(
                                        new SleepAction(LL_MIN_PICTURE_TIME),
                                        new FirstTerminateAction(
                                                sampleAligner.detectTarget(),
                                                new SleepAction(LL_MAX_PICTURE_TIME)
                                        )
                                );
                            }
                            break;
                        case TAKING_PICTURE:

                            boolean done = snapshotAction.run(p);

                            EditablePose offset = new EditablePose(sampleAligner.getTargetOffset());

                            if (!(offset.x == 0 && offset.y == 0 && offset.heading == 0)) {

                                targetOffset = offset;
                                double extendoInches = hypot(targetOffset.x, targetOffset.y) + LL_EXTEND_OFFSET;

                                robot.intake.extendo.powerCap = LL_SPEED_MAX_EXTENDO;

                                activeTraj = robot.drivetrain.actionBuilder(robot.drivetrain.pose)
                                        .turn(-targetOffset.heading * LL_TURN_MULTIPLIER)
                                        .stopAndAdd(() -> {
                                            robot.intake.extendo.setTarget(extendoInches);
                                            robot.intake.setAngle(0.01);
                                            robot.intake.setRoller(0.001);
                                        })
                                        .stopAndAdd(new FirstTerminateAction(
                                                t -> robot.intake.extendo.getPosition() < extendoInches - LL_DISTANCE_START_LOWERING,
                                                new SleepAction(1)
                                        ))
                                        .stopAndAdd(() -> robot.intake.setRoller(1))
                                        .stopAndAdd(timer::reset)
                                        .afterTime(0, t -> !robot.intake.setAngle(timer.seconds() * LL_ANGLE_BUCKET_INCREMENT))
                                        .waitSeconds(LL_WAIT_INTAKE)
                                        .build();

                                state = SUB_INTAKING;

                            } else if (done) searchAgainForSample(robot);

                            break;

                        case SUB_INTAKING:


                            if (remaining < TIME_PARK) {

                                Pose2d current = robot.drivetrain.pose;

                                EditablePose zone = barnacle == 1 ? park1 :
                                                    barnacle == 2 ? park2 :
                                                                    park3;

                                activeTraj = robot.drivetrain.actionBuilder(current)
                                        .stopAndAdd(() -> robot.intake.setRollerAndAngle(0))
                                        .setTangent(PI + current.heading.toDouble())
                                        .waitSeconds(WAIT_INTAKE_RETRACT_POST_SUB)
                                        .splineTo(zone.toVector2d(), PI + zone.heading)
                                        .build();
                                state = PARKING;
                                stopDt();

                            } else if (robot.hasSample()) {
                                robot.headlight.setActivated(false);
                                Pose2d current = robot.drivetrain.pose;

                                robot.deposit.requireDistBeforeLoweringLift = true;
                                robot.intake.retractBucketBeforeExtendo = true;
                                robot.intake.extendo.powerCap = 1;
                                robot.deposit.setWristPitchingAngle(0);

                                activeTraj = robot.drivetrain.actionBuilder(current)
                                        .stopAndAdd(() -> robot.intake.setRollerAndAngle(0))
                                        .setTangent(PI + current.heading.toDouble())
                                        .waitSeconds(WAIT_INTAKE_RETRACT_POST_SUB)
                                        .splineTo(scoringFromSub.toVector2d(), PI + scoringFromSub.heading)
                                        .afterTime(0, () -> robot.deposit.setWristPitchingAngle(ANGLE_PITCH_FROM_SUB))
                                        .stopAndAdd(scoreSample(robot))
                                        .afterTime(0, () -> robot.deposit.setWristPitchingAngle(0))
                                        .build();

                                subCycle++;
                                state = SCORING;
                                stopDt();

                            } else if (trajDone) searchAgainForSample(robot);

                            break;

                        case PARKING:
                            robot.deposit.lvl1Ascent = true;
                            return !trajDone;
                    }

                    return true;
                }

                private void searchAgainForSample(Robot robot) {
                    robot.intake.setRollerAndAngle(0);
                    robot.intake.extendo.setExtended(false);
                    robot.intake.ejectSample();

                    Pose2d current = robot.drivetrain.pose;
                    activeTraj = new FirstTerminateAction(
                            t -> !(robot.intake.extendo.getPosition() <= 2),
                            new SleepAction(WAIT_MAX_BEFORE_RE_SEARCH)
                    );
//                            robot.drivetrain.actionBuilder(current)
//                            .setTangent(PI / 2)
//                            .strafeToLinearHeading(snapshotPos.toVector2d(), snapshotPos.heading)
//                            .build();

                    state = DRIVING_TO_SUB;
                }
            };
        }

        // Parallel action to bulk read, update trajectory, and update robot (robot.run())
        ParallelAction auton = new ParallelAction(
                telemetryPacket -> {
                    robot.bulkReader.bulkRead();
                    return opModeIsActive();
                },
                trajectory,
                telemetryPacket -> {
                    pose = robot.drivetrain.pose;
                    robot.run();
                    return opModeIsActive();
                }
        );

        printConfig(true, 0, selection, specimenSide, cycles);
        mTelemetry.update();

        waitForStart(); //--------------------------------------------------------------------------------------------------------------------------

        robot.drivetrain.pinpoint.setPositionRR(pose);

        Actions.runBlocking(auton);

        if (robot.deposit.hasSample()) Tele.holdingSample = true;

        Thread.sleep((long) (DEAD_TIME * 1000));
    }

    private static void printConfig(boolean confirmed, double t, AutonConfig selection, boolean specimenSide, int cycles) {
        mTelemetry.addLine(confirmed ?
                "AUTONOMOUS READY" :
                "Confirm configuration (confirming in " + (int) ceil(5 - t) + " seconds)" + selection.markIf(CONFIRMING)
        );
        mTelemetry.addLine();
        mTelemetry.addLine();
        mTelemetry.addLine((isRedAlliance ? "RED" : "BLUE") + " alliance" + selection.markIf(EDITING_ALLIANCE));
        mTelemetry.addLine();
        if (specimenSide) {
            mTelemetry.addLine("RIGHT (observation-side)" + selection.markIf(EDITING_SIDE));
            mTelemetry.addLine();
            mTelemetry.addLine((cycles + 1) + "+0 (" + cycles + " from observation zone)" + selection.markIf(EDITING_CYCLES));

        } else mTelemetry.addLine("LEFT (basket-side)" + selection.markIf(EDITING_SIDE));

        mTelemetry.update();
    }

    private static Action scoreSpecimen(Robot robot) {
        return new SequentialAction(
                new SleepAction(WAIT_APPROACH_CHAMBER),
                telemetryPacket -> !(robot.deposit.state == Deposit.State.AT_CHAMBER && robot.deposit.lift.atPosition(HEIGHT_CHAMBER_HIGH)), // wait until deposit in position
                new InstantAction(robot.deposit::nextState),
                telemetryPacket -> robot.deposit.hasSample(), // wait until spec scored
                new SleepAction(WAIT_SCORE_CHAMBER)
        );
    }

    private static Action scoreSample(Robot robot) {
        return new SequentialAction(
                new InstantAction(() -> {
                    if (!robot.hasSample()) robot.intake.transfer(NEUTRAL);
                }),
                new SleepAction(WAIT_APPROACH_BASKET),
                t -> !(robot.deposit.basketReady() && abs(robot.deposit.lift.getPosition() - HEIGHT_BASKET_HIGH) <= LIFT_HEIGHT_TOLERANCE),
                new InstantAction(robot.deposit::nextState),
                new SleepAction(WAIT_SCORE_BASKET)
        );
    }

    private static Action preExtend(Robot robot, double length) {
        return new SequentialAction(
                t -> robot.intake.hasSample(),
                t -> robot.deposit.state.ordinal() < Deposit.State.ARM_MOVING_TO_BASKET.ordinal(),
                new InstantAction(() -> {
                    robot.intake.extendo.setTarget(length);
                    robot.intake.setRollerAndAngle(1);
                })
        );
    }

    private static boolean atPose(Robot robot, EditablePose target) {
        EditablePose current = new EditablePose(robot.drivetrain.pose);
        PoseVelocity2d velocityRR = robot.drivetrain.pinpoint.getVelocityRR();
        return  abs(target.x - current.x) <= basketError.x &&
                abs(target.y - current.y) <= basketError.y &&
                abs(target.heading - current.heading) <= basketError.heading &&
                abs(velocityRR.linearVel.x) <= basketVelError.x &&
                abs(velocityRR.linearVel.y) <= basketVelError.y &&
                abs(velocityRR.angVel) <= basketVelError.heading;

    }

}
