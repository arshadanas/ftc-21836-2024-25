package org.firstinspires.ftc.teamcode.opmode;

import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.A;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.B;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_DOWN;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_LEFT;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_RIGHT;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_UP;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.LEFT_BUMPER;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.RIGHT_BUMPER;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.X;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.Y;
import static org.firstinspires.ftc.teamcode.opmode.Auto.scoring;
import static org.firstinspires.ftc.teamcode.opmode.Auto.chamberRight;
import static org.firstinspires.ftc.teamcode.opmode.Tele.TeleOpConfig.EDITING_ALLIANCE;
import static org.firstinspires.ftc.teamcode.opmode.Tele.TeleOpConfig.EDITING_FIELD_CENTRIC;
import static org.firstinspires.ftc.teamcode.opmode.Tele.TeleOpConfig.EDITING_SLOW_LOCK;
import static org.firstinspires.ftc.teamcode.opmode.Tele.TeleOpConfig.PRELOAD_SAMPLE;
import static org.firstinspires.ftc.teamcode.opmode.Tele.TeleOpConfig.PRELOAD_SPECIMEN;
import static org.firstinspires.ftc.teamcode.opmode.Auto.isRedAlliance;
import static org.firstinspires.ftc.teamcode.opmode.Auto.mTelemetry;
import static org.firstinspires.ftc.teamcode.opmode.Auto.pose;
import static org.firstinspires.ftc.teamcode.subsystem.Deposit.Position.FLOOR;
import static org.firstinspires.ftc.teamcode.subsystem.Deposit.Position.HIGH;
import static org.firstinspires.ftc.teamcode.subsystem.Deposit.Position.LOW;
import static org.firstinspires.ftc.teamcode.control.vision.pipeline.Sample.NEUTRAL;
import static org.firstinspires.ftc.teamcode.subsystem.utility.LEDIndicator.State.GREEN;
import static org.firstinspires.ftc.teamcode.subsystem.utility.LEDIndicator.State.OFF;
import static java.lang.Math.PI;
import static java.lang.Math.sin;
import static java.lang.Math.toDegrees;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.control.motion.EditablePose;
import org.firstinspires.ftc.teamcode.control.motion.PIDDriver;
import org.firstinspires.ftc.teamcode.subsystem.Robot;
import org.firstinspires.ftc.teamcode.subsystem.utility.LEDIndicator;

@TeleOp
public final class Tele extends LinearOpMode {

    public static boolean holdingSample = false;

    public static double TIME_CLIMB_INDICATOR_ON = 0.05;

    enum TeleOpConfig {
        PRELOAD_SAMPLE,
        PRELOAD_SPECIMEN,
        EDITING_ALLIANCE,
        EDITING_SLOW_LOCK,
        EDITING_FIELD_CENTRIC;

        public static final TeleOpConfig[] selections = values();

        public TeleOpConfig plus(int i) {
            int max = selections.length;
            return selections[((ordinal() + i) % max + max) % max];
        }
        public String markIf(TeleOpConfig s) {
            return this == s ? " <" : "";
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {

        ElapsedTime matchTimer = new ElapsedTime();

        double TELE = 120; // seconds
        double CLIMB_TIME = TELE - 15; // 15 seconds for climb

        mTelemetry = telemetry;
//                new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        Robot robot = new Robot(hardwareMap, pose);
        // robot.drivetrain.trackHeadingOnly = true;

        LEDIndicator indicator = new LEDIndicator(hardwareMap, "green", "red");

        PIDDriver driver = new PIDDriver();

        GamepadEx gamepadEx1 = new GamepadEx(gamepad1);

        TeleOpConfig selection = PRELOAD_SAMPLE;

        boolean slowModeLocked = false, useFieldCentric = true, doTelemetry = false;

        if (holdingSample) robot.deposit.preloadSample();

        while (opModeInInit()) {
            gamepadEx1.readButtons();

            if (gamepadEx1.wasJustPressed(DPAD_UP)) {
               do
                    selection = selection.plus(-1);
               while (robot.deposit.hasSample() && (selection == PRELOAD_SAMPLE || selection == PRELOAD_SPECIMEN));
            } else if (gamepadEx1.wasJustPressed(DPAD_DOWN)) {
               do
                    selection = selection.plus(1);
               while (robot.deposit.hasSample() && (selection == PRELOAD_SAMPLE || selection == PRELOAD_SPECIMEN));
            }

            switch (selection) {
                case PRELOAD_SAMPLE:
                    if (gamepadEx1.wasJustPressed(X)) {
                       robot.deposit.preloadSample();
                       selection = selection.plus(2);
                    }
                    break;
                case PRELOAD_SPECIMEN:
                    if (gamepadEx1.wasJustPressed(X)) {
                       robot.deposit.preloadSpecimen();
                       selection = selection.plus(1);
                    }
                    break;
                case EDITING_ALLIANCE:
                    if (gamepadEx1.wasJustPressed(X)) isRedAlliance = !isRedAlliance;
                    break;
                case EDITING_SLOW_LOCK:
                    if (gamepadEx1.wasJustPressed(X)) slowModeLocked = !slowModeLocked;
                    break;
                case EDITING_FIELD_CENTRIC:
                    if (gamepadEx1.wasJustPressed(X)) useFieldCentric = !useFieldCentric;
                    break;
            }

            robot.drivetrain.setHeadingWithStick(gamepadEx1.getRightX(), gamepadEx1.getRightY());
            robot.drivetrain.updatePoseEstimate();

            if (robot.deposit.hasSample()) mTelemetry.addLine("Preloaded a " + (robot.deposit.chamberReady() ? "SPECIMEN" : "SAMPLE"));
            else
            {
                mTelemetry.addLine("Preload sample" + selection.markIf(PRELOAD_SAMPLE));
                mTelemetry.addLine();
                mTelemetry.addLine("Preload specimen" + selection.markIf(PRELOAD_SPECIMEN));
            }
            mTelemetry.addLine();
            mTelemetry.addLine((isRedAlliance ? "RED" : "BLUE") + " alliance" + selection.markIf(EDITING_ALLIANCE));
            mTelemetry.addLine();
            mTelemetry.addData("Slow mode", (slowModeLocked ? "LOCKED" : "unlocked") + selection.markIf(EDITING_SLOW_LOCK));
            mTelemetry.addLine();
            mTelemetry.addLine((useFieldCentric ? "Field centric" : "ROBOT CENTRIC") + " driving" + selection.markIf(EDITING_FIELD_CENTRIC));
            mTelemetry.addLine();
            mTelemetry.addData("Heading (deg, set with right stick)", toDegrees(robot.drivetrain.pose.heading.toDouble()));

            mTelemetry.update();
        }

// ----------------------------------------------------------------------------------------------------------------------------------------------------------------

        robot.intake.setAlliance(isRedAlliance);

        matchTimer.reset();

        boolean extended = true;

        // Control loop:
        while (opModeIsActive()) {
            // Read sensors + gamepads:
            robot.bulkReader.bulkRead();
            robot.drivetrain.updatePoseEstimate();
            gamepadEx1.readButtons();

            if (!robot.intake.extendo.isExtended()) extended = false;

            double triggers = gamepad1.right_trigger - gamepad1.left_trigger;

            if (gamepadEx1.isDown(LEFT_BUMPER)) {

                extended = false;

                robot.intake.setRollerAndAngle(0);
                robot.deposit.setWristPitchingAngle(0);
                robot.intake.extendo.runManual(triggers);
                robot.deposit.lift.runManual(gamepadEx1.getLeftY() * (gamepadEx1.isDown(RIGHT_BUMPER) ? 0.3 : 1));

                robot.drivetrain.setHeadingWithStick(gamepadEx1.getRightX(), gamepadEx1.getRightY());

                if (!robot.deposit.lift.gearSwitch.isActivated())
                    robot.drivetrain.run(0, 0, 0, false, true);

                if (gamepadEx1.wasJustPressed(Y)) robot.deposit.lift.hold = !robot.deposit.lift.hold;
                if (gamepadEx1.wasJustPressed(X)) doTelemetry = !doTelemetry;
                if (gamepadEx1.wasJustPressed(B)) robot.deposit.lift.gearSwitch.toggle();
                if (gamepadEx1.wasJustPressed(A)) robot.deposit.lift.tilt.toggle();

                if (gamepadEx1.wasJustPressed(DPAD_RIGHT))          robot.deposit.goToBasket();
                // else if (gamepadEx1.wasJustPressed(DPAD_UP))        
                else if (gamepadEx1.wasJustPressed(DPAD_LEFT))      robot.deposit.preloadSpecimen();
                else if (gamepadEx1.wasJustPressed(DPAD_DOWN))     robot.intake.specimenMode = !robot.intake.specimenMode;

            } else {

                robot.intake.setRollerAndAngle(robot.deposit.hasSample() ? 0 : triggers);
                robot.deposit.setWristPitchingAngle(robot.deposit.hasSample() ? triggers : 0);
                robot.intake.extendo.runManual(extended && robot.intake.extendoRetracting() ? -1 : 0);
                if (!robot.deposit.lift.isClimbing()) robot.deposit.lift.runManual(0);

                if (gamepadEx1.wasJustPressed(X)) driver.reset();

                if (!robot.deposit.lift.gearSwitch.isActivated())
                    robot.drivetrain.run(
                            gamepadEx1.getLeftX(),
                            gamepadEx1.getLeftY(),
                            gamepadEx1.isDown(X) && (robot.deposit.basketReady() || robot.intake.hasSample() || robot.deposit.intaking())?
                                    driver.driveTo(
                                            new EditablePose(robot.drivetrain.pose),
                                            robot.deposit.intaking() ? chamberRight : scoring
                                    ).drivePower.heading :
                                    gamepadEx1.getRightX(),
                            slowModeLocked || gamepadEx1.isDown(RIGHT_BUMPER) || triggers > 0,
                            useFieldCentric
                    );

                if (gamepadEx1.wasJustPressed(Y)) robot.deposit.lift.climb();
                // if (gamepadEx1.wasJustPressed(X)) doTelemetry = !doTelemetry;
                if (gamepadEx1.wasJustPressed(B)) robot.deposit.nextState();
//                if (gamepadEx1.wasJustPressed(A)) robot.headlight.toggle();

                robot.headlight.setActivated(robot.intake.extendo.isExtended() || gamepad1.cross);
    
                if (gamepadEx1.wasJustPressed(DPAD_RIGHT))          robot.intake.transfer(NEUTRAL);
                else if (gamepadEx1.wasJustPressed(DPAD_UP))        robot.deposit.setPosition(HIGH);
                else if (gamepadEx1.wasJustPressed(DPAD_LEFT))      robot.deposit.setPosition(LOW);
                else if (gamepadEx1.wasJustPressed(DPAD_DOWN))      robot.deposit.setPosition(FLOOR);

            }

            if (gamepad1.touchpad_finger_1) {
                robot.intake.extendo.setWithTouchpad(gamepad1.touchpad_finger_1_x);
            }

            robot.run();

            if (doTelemetry) {
                robot.printTelemetry();
                mTelemetry.update();
            }

            double t = matchTimer.seconds();
            indicator.setState(t >= CLIMB_TIME ?
                    sin(PI * t / TIME_CLIMB_INDICATOR_ON) >= 0 ? GREEN : OFF :
                    robot.intake.hasSample() ? GREEN : OFF
            );
        }
    }
}
