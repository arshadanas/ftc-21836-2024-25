package org.firstinspires.ftc.teamcode.opmode.mechanismtest;

import static org.firstinspires.ftc.teamcode.control.vision.AutoSampleAligner.LL_TURN_MULTIPLIER;
import static org.firstinspires.ftc.teamcode.opmode.Auto.LL_ANGLE_BUCKET_INCREMENT;
import static org.firstinspires.ftc.teamcode.opmode.Auto.LL_DISTANCE_START_LOWERING;
import static org.firstinspires.ftc.teamcode.opmode.Auto.LL_EXTEND_OFFSET;
import static org.firstinspires.ftc.teamcode.opmode.Auto.LL_MAX_PICTURE_TIME;
import static org.firstinspires.ftc.teamcode.opmode.Auto.LL_MIN_PICTURE_TIME;
import static org.firstinspires.ftc.teamcode.opmode.Auto.LL_SPEED_MAX_EXTENDO;
import static org.firstinspires.ftc.teamcode.opmode.Auto.LL_SWEEP_ANGLE_RANGE;
import static org.firstinspires.ftc.teamcode.opmode.Auto.LL_SWEEP_SPEED;
import static org.firstinspires.ftc.teamcode.opmode.Auto.LL_WAIT_INTAKE;
import static org.firstinspires.ftc.teamcode.opmode.Auto.mTelemetry;

import static java.lang.Math.hypot;
import static java.lang.Math.toDegrees;
import static java.lang.Math.toRadians;

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
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.control.FirstTerminateAction;
import org.firstinspires.ftc.teamcode.control.motion.EditablePose;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.control.vision.AutoSampleAligner;
import org.firstinspires.ftc.teamcode.control.vision.LimelightEx;
import org.firstinspires.ftc.teamcode.subsystem.Robot;

@Config
@TeleOp(group = "Single mechanism test")
public class TestLimelightSample extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        mTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        Robot robot = new Robot(hardwareMap, new Pose2d(0, 0, 0));
        robot.headlight.toggle();
        robot.intake.specimenMode = true;

        Limelight3A limelight3a = hardwareMap.get(Limelight3A.class, "limelight");
        AutoSampleAligner sampleAligner = new AutoSampleAligner(new LimelightEx(limelight3a, hardwareMap));
        sampleAligner.activateLimelight(AutoSampleAligner.Pipeline.YELLOW_BLUE);
        limelight3a.stop();
        limelight3a.start();

        waitForStart();

        // after init
        Actions.runBlocking(new SequentialAction(
                new SleepAction(LL_MIN_PICTURE_TIME),
                new FirstTerminateAction(
                        sampleAligner.detectTarget(),
                        new SleepAction(LL_MAX_PICTURE_TIME)
                )
        ));

        EditablePose targetOffset = new EditablePose(sampleAligner.getTargetOffset());
        double extendoInches = hypot(targetOffset.x, targetOffset.y) + LL_EXTEND_OFFSET;

        mTelemetry.addData("dx (in)", targetOffset.x);
        mTelemetry.addData("dy (in)", targetOffset.y);
        mTelemetry.addData("d(theta) (deg)", toDegrees(targetOffset.heading));
        mTelemetry.addData("Extension (in)", extendoInches);
        mTelemetry.addData("X degrees", sampleAligner.measuredXDegreesDiff);
        mTelemetry.addData("Y degrees", sampleAligner.measuredYDegreesDiff);
        mTelemetry.update();

        ElapsedTime timer = new ElapsedTime();

        TurnConstraints llSweepConstraint = new TurnConstraints(LL_SWEEP_SPEED, -MecanumDrive.PARAMS.maxAngAccel, MecanumDrive.PARAMS.maxAngAccel);

        robot.intake.extendo.powerCap = LL_SPEED_MAX_EXTENDO;

        Action traj = robot.drivetrain.actionBuilder(robot.drivetrain.pose)
                .turn(-targetOffset.heading * LL_TURN_MULTIPLIER)
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
