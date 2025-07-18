package org.firstinspires.ftc.teamcode.opmode.mechanismtest;

import static org.firstinspires.ftc.teamcode.opmode.Auto.LL_ANGLE_BUCKET_INCREMENT;
import static org.firstinspires.ftc.teamcode.opmode.Auto.LL_DISTANCE_START_LOWERING;
import static org.firstinspires.ftc.teamcode.opmode.Auto.LL_EXTEND_OFFSET;
import static org.firstinspires.ftc.teamcode.opmode.Auto.LL_MAX_PICTURE_TIME;
import static org.firstinspires.ftc.teamcode.opmode.Auto.LL_MIN_PICTURE_TIME;
import static org.firstinspires.ftc.teamcode.opmode.Auto.LL_SPEED_MAX_EXTENDO;
import static org.firstinspires.ftc.teamcode.opmode.Auto.LL_SWEEP_SPEED;
import static org.firstinspires.ftc.teamcode.opmode.Auto.LL_TURN_MULTIPLIER;
import static org.firstinspires.ftc.teamcode.opmode.Auto.LL_WAIT_INTAKE;
import static org.firstinspires.ftc.teamcode.opmode.Auto.mTelemetry;

import static java.lang.Math.hypot;
import static java.lang.Math.toDegrees;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TurnConstraints;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.control.FirstTerminateAction;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.control.vision.detector.SampleDetector;
import org.firstinspires.ftc.teamcode.subsystem.Robot;

@Config
@TeleOp(group = "Single mechanism test")
public class TestSampleDetector extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        mTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        Robot robot = new Robot(hardwareMap, new Pose2d(0, 0, 0));
        robot.headlight.toggle();
        robot.intake.specimenMode = true;

        SampleDetector sampleDetector = new SampleDetector(hardwareMap);
        sampleDetector.setPipeline(SampleDetector.Pipeline.YELLOW_BLUE);

        waitForStart();

        // after init
        Actions.runBlocking(new ParallelAction(
                telemetryPacket -> {
                    robot.bulkReader.bulkRead();
                    return opModeIsActive();
                },
                new SequentialAction(
                        new SleepAction(LL_MIN_PICTURE_TIME),
                        new FirstTerminateAction(
                                t -> !sampleDetector.run(),
                                new SleepAction(LL_MAX_PICTURE_TIME)
                        )
                ),
                telemetryPacket -> {
                    robot.run();
                    return opModeIsActive();
                }
        ));

        double extendoInches = hypot(sampleDetector.offsetToSample.x, sampleDetector.offsetToSample.y) + LL_EXTEND_OFFSET;

        mTelemetry.addData("xDegFromLens", sampleDetector.xDegFromLens);
        mTelemetry.addData("yDegFromLens", sampleDetector.yDegFromLens);
        mTelemetry.addData("xDistFromCenter (in)", sampleDetector.offsetToSample.x);
        mTelemetry.addData("yDistFromCenter (in)", sampleDetector.offsetToSample.y);
        mTelemetry.addData("targetOffset turn (deg)", toDegrees(sampleDetector.offsetToSample.heading));
        mTelemetry.addData("Extension (in)", extendoInches);
        mTelemetry.update();

        ElapsedTime timer = new ElapsedTime();

        TurnConstraints llSweepConstraint = new TurnConstraints(LL_SWEEP_SPEED, -MecanumDrive.PARAMS.maxAngAccel, MecanumDrive.PARAMS.maxAngAccel);

        robot.intake.extendo.powerCap = LL_SPEED_MAX_EXTENDO;

        Action traj = robot.drivetrain.actionBuilder(robot.drivetrain.pose)
                .turn(-sampleDetector.offsetToSample.heading * LL_TURN_MULTIPLIER)
                .stopAndAdd(() -> {
                    robot.intake.extendo.setTarget(extendoInches);
                    robot.intake.setAngle(0.01);
                })
                .stopAndAdd(new FirstTerminateAction(
                        t -> robot.intake.extendo.getPosition() < extendoInches - LL_DISTANCE_START_LOWERING,
                        new SleepAction(1)
                ))
                .stopAndAdd(() -> robot.intake.setRoller(1))
                .stopAndAdd(timer::reset)
                .afterTime(0, t -> !robot.intake.setAngle(timer.seconds() * LL_ANGLE_BUCKET_INCREMENT))
                .waitSeconds(LL_WAIT_INTAKE)

                .stopAndAdd(t -> !robot.hasSample())
                .stopAndAdd(() -> robot.intake.setRollerAndAngle(0))
                .lineToX(-5)
                .build();

        Actions.runBlocking(new ParallelAction(
                telemetryPacket -> {
                    robot.bulkReader.bulkRead();
                    return opModeIsActive();
                },
                traj,
                telemetryPacket -> {
                    robot.run();
                    return opModeIsActive();
                }
        ));



    }
}
