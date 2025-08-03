package org.firstinspires.ftc.teamcode.subsystem;

import static com.qualcomm.robotcore.util.Range.clip;
import static org.firstinspires.ftc.teamcode.opmode.Auto.chamberRight;
import static org.firstinspires.ftc.teamcode.opmode.Auto.divider;
import static org.firstinspires.ftc.teamcode.opmode.Auto.mTelemetry;
import static org.firstinspires.ftc.teamcode.subsystem.Deposit.Position.FLOOR;
import static org.firstinspires.ftc.teamcode.subsystem.Deposit.Position.HIGH;
import static org.firstinspires.ftc.teamcode.subsystem.Deposit.Position.LOW;
import static org.firstinspires.ftc.teamcode.subsystem.Deposit.State.BASKET_TO_STANDBY;
import static org.firstinspires.ftc.teamcode.subsystem.Deposit.State.COUNTER_ROLLING;
import static org.firstinspires.ftc.teamcode.subsystem.Deposit.State.ENTERING_BUCKET;
import static org.firstinspires.ftc.teamcode.subsystem.Deposit.State.EXITING_BUCKET;
import static org.firstinspires.ftc.teamcode.subsystem.Deposit.State.FALLING_BASKET;
import static org.firstinspires.ftc.teamcode.subsystem.Deposit.State.GRABBING_SPECIMEN;
import static org.firstinspires.ftc.teamcode.subsystem.Deposit.State.AT_BASKET;
import static org.firstinspires.ftc.teamcode.subsystem.Deposit.State.ARM_MOVING_TO_BASKET;
import static org.firstinspires.ftc.teamcode.subsystem.Deposit.State.LIFT_MOVING_TO_BASKET;
import static org.firstinspires.ftc.teamcode.subsystem.Deposit.State.RAISED_TO_STANDBY;
import static org.firstinspires.ftc.teamcode.subsystem.Deposit.State.RELEASED_SPEC_TO_STANDBY;
import static org.firstinspires.ftc.teamcode.subsystem.Deposit.State.RELEASING_SPECIMEN;
import static org.firstinspires.ftc.teamcode.subsystem.Deposit.State.AT_CHAMBER;
import static org.firstinspires.ftc.teamcode.subsystem.Deposit.State.INTAKING_SPECIMEN;
import static org.firstinspires.ftc.teamcode.subsystem.Deposit.State.RAISING_SPECIMEN;
import static org.firstinspires.ftc.teamcode.subsystem.Deposit.State.STANDBY_TO_CHAMBER;
import static org.firstinspires.ftc.teamcode.subsystem.Deposit.State.CLAW_CLOSING;
import static org.firstinspires.ftc.teamcode.subsystem.Deposit.State.WAITING_FOR_BUCKET;
import static org.firstinspires.ftc.teamcode.subsystem.utility.cachedhardware.CachedSimpleServo.getAxon;

import static java.lang.Math.abs;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.control.motion.EditablePose;
import org.firstinspires.ftc.teamcode.opmode.Auto;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystem.utility.cachedhardware.CachedSimpleServo;

@Config
public final class Deposit {

    public static double
            ANGLE_CLAW_INTAKING_SPECIMEN = 170,
            ANGLE_CLAW_TRANSFER = 190,
            ANGLE_CLAW_MOVING_TO_SPECIMEN = 229.7,
            ANGLE_CLAW_SAMPLE = 215,
            ANGLE_CLAW_DROPPING_SAMPLE = 170,
            ANGLE_CLAW_SPECIMEN = 292.353,

            HEIGHT_ABOVE_INTAKE = 10,
            HEIGHT_BASKET_LOW = 9,
            HEIGHT_BASKET_HIGH = 25,
            INCREMENT_REACH_ABOVE_BASKET = 1,
            PASSIVE_INCREMENT = (26.5 - HEIGHT_BASKET_HIGH)/13.0,
            HEIGHT_INTAKING_SPECIMEN = 0,

            AT_BASKET_TOLERANCE = 10,

            HEIGHT_CHAMBER_HIGH = 13,
            HEIGHT_CHAMBER_LOW = 0,

            ANGLE_WRIST_PITCH = 104,

            TIME_ENTERING_BUCKET = .075,
            TIME_COUNTER_ROLLING = 0.15,
            TIME_CLAW_CLOSING = .15,
            TIME_EXITING_BUCKET = 0,
            TIME_TO_BASKET = 0.38,
            TIME_MAX_SAMPLE_RELEASE = 1,
            TIME_SAMPLE_RELEASE = .125,
            TIME_BASKET_TO_STANDBY = .38,

            TIME_WAIT_FOR_BUCKET = 0.15,
            TIME_TO_INTAKING_SPEC = 0.125,
            TIME_SPEC_GRAB = 0.2,
            TIME_RAISE_SPEC = 0.5,
            TIME_RAISED_SPEC_TO_STANDBY = 0.25,
            TIME_STANDBY_TO_CHAMBER = 0.5,
            TIME_MAX_SPEC_RELEASE = 1,
            TIME_RELEASED_SPEC_TO_STANDBY = 0.1,

            Y_DIST_FROM_CHAMBER_RETRACT = -3;

    public static EditablePose distFromBasketLiftDown = new EditablePose(1, 1, 0);

    public static ArmPosition
            ASCENT =        new ArmPosition(165, 100),
            BASKET =        new ArmPosition(313, 150),
            BASKET_STEEP =  new ArmPosition(260, 215),
            CHAMBER =       new ArmPosition(150, 80),
            MOVING_TO_INTAKING_SPEC = new ArmPosition(18, 55),
            INTAKING_SPEC = new ArmPosition(18, 55),
            IN_INTAKE =     new ArmPosition(95, 70),
            RAISED_SPEC =   new ArmPosition(INTAKING_SPEC.arm, 20),
            STANDBY =       new ArmPosition(120, 35);

    public enum State {
        STANDBY         (Deposit.STANDBY),

        ENTERING_BUCKET (IN_INTAKE),
        COUNTER_ROLLING (IN_INTAKE),
        CLAW_CLOSING(IN_INTAKE),
        EXITING_BUCKET  (IN_INTAKE),
        LIFT_MOVING_TO_BASKET (IN_INTAKE),
        ARM_MOVING_TO_BASKET(BASKET),
        AT_BASKET           (BASKET),
        FALLING_BASKET      (BASKET),
        BASKET_TO_STANDBY   (Deposit.STANDBY),

        WAITING_FOR_BUCKET      (Deposit.STANDBY),
        MOVING_TO_INTAKING_SPEC (Deposit.MOVING_TO_INTAKING_SPEC),
        INTAKING_SPECIMEN       (INTAKING_SPEC),
        GRABBING_SPECIMEN       (INTAKING_SPEC),
        RAISING_SPECIMEN        (RAISED_SPEC),
        RAISED_TO_STANDBY       (Deposit.STANDBY),
        STANDBY_TO_CHAMBER      (CHAMBER),
        AT_CHAMBER              (CHAMBER),
        RELEASING_SPECIMEN      (CHAMBER),
        RELEASED_SPEC_TO_STANDBY(Deposit.STANDBY);

        private final ArmPosition armPosition;

        State(ArmPosition armPosition) {
            this.armPosition = armPosition;
        }

        private static final State[] states = values();

        private State next() {
            int max = states.length;
            return states[((ordinal() + 1) % max + max) % max];
        }
    }

    public enum Position {
        FLOOR,
        LOW,
        HIGH,
    }

    public final Lift lift;
    public final CachedSimpleServo claw;
    private final CachedSimpleServo wrist, armR, armL;

    public boolean lvl1Ascent = false, requireDistBeforeLoweringLift = true, steepArm = false;

    public final ElapsedTime timer = new ElapsedTime();

    public Deposit.State state = Deposit.State.STANDBY;

    private boolean topRung = true;
    private double sampleHeight = HEIGHT_BASKET_HIGH, wristPitchingAngle = 0, lastChamberY = chamberRight.y;

    private final MecanumDrive dt;
    private Pose2d lastBasketPos = Auto.scoring.toPose2d();

    Deposit(HardwareMap hardwareMap, MecanumDrive dt) {
        lift = new Lift(hardwareMap, this.dt = dt, this::goToBasket);
        claw = getAxon(hardwareMap, "claw").reversed();
        armR = getAxon(hardwareMap, "arm right");
        armL = getAxon(hardwareMap, "arm left").reversed();
        wrist = getAxon(hardwareMap, "wrist").reversed();
    }

    void run() {

        switch (state) {
            case ENTERING_BUCKET:
                if (timer.seconds() >= TIME_ENTERING_BUCKET) nextState();
                break;
            case COUNTER_ROLLING:
                if (timer.seconds() >= TIME_COUNTER_ROLLING) nextState();
                break;
            case CLAW_CLOSING:
                if (timer.seconds() >= TIME_CLAW_CLOSING) nextState();
                break;
            case EXITING_BUCKET:
                if (timer.seconds() >= TIME_EXITING_BUCKET) nextState();
                break;
            case LIFT_MOVING_TO_BASKET:
                if (abs(lift.getTarget() - lift.getPosition()) <= AT_BASKET_TOLERANCE || timer.seconds() >= 2.5) nextState();
                break;
            case ARM_MOVING_TO_BASKET:
                if (timer.seconds() >= TIME_TO_BASKET) nextState();
                break;
            case FALLING_BASKET:

                boolean farEnoughFromBasket =   dt.pose.position.x - lastBasketPos.position.x > distFromBasketLiftDown.x &&
                        dt.pose.position.y - lastBasketPos.position.y > distFromBasketLiftDown.y;

                double t = timer.seconds();
                if (
                        t >= TIME_MAX_SAMPLE_RELEASE ||
                        farEnoughFromBasket ||
                        !requireDistBeforeLoweringLift && t >= TIME_SAMPLE_RELEASE
                ) nextState();
                break;
            case BASKET_TO_STANDBY:
                if (timer.seconds() >= TIME_BASKET_TO_STANDBY) nextState();
                break;
            case WAITING_FOR_BUCKET:
                if (!requestingIntakeToMove() || timer.seconds() >= TIME_WAIT_FOR_BUCKET) nextState();
                break;
            case MOVING_TO_INTAKING_SPEC:
                if (timer.seconds() >= TIME_TO_INTAKING_SPEC) nextState();
                break;
            case GRABBING_SPECIMEN:
                if (timer.seconds() >= TIME_SPEC_GRAB) nextState();
                break;
            case RAISING_SPECIMEN:
                if (timer.seconds() >= TIME_RAISE_SPEC) nextState();
                break;
            case RAISED_TO_STANDBY:
                if (timer.seconds() >= TIME_RAISED_SPEC_TO_STANDBY) nextState();
                break;
            case STANDBY_TO_CHAMBER:
                if (timer.seconds() >= TIME_STANDBY_TO_CHAMBER) nextState();
                break;
            case RELEASING_SPECIMEN:
                boolean farEnoughFromChamber = dt.pose.position.y - lastChamberY < Y_DIST_FROM_CHAMBER_RETRACT;
                if (farEnoughFromChamber || timer.seconds() >= TIME_MAX_SPEC_RELEASE) nextState();
                break;
            case RELEASED_SPEC_TO_STANDBY:
                if (timer.seconds() >= TIME_RELEASED_SPEC_TO_STANDBY) state = State.STANDBY;
                break;
        }

        lift.run();

        ArmPosition armPosition =
                state.armPosition == BASKET && steepArm ? BASKET_STEEP :
                state == State.STANDBY && lvl1Ascent ? ASCENT :
                state.armPosition;

        armR.turnToAngle(armPosition.arm);
        armL.turnToAngle(armPosition.arm);
        wrist.turnToAngle(armPosition.wrist + (state.armPosition == BASKET ? wristPitchingAngle * ANGLE_WRIST_PITCH : 0));

        claw.turnToAngle(
                state == State.STANDBY ? ANGLE_CLAW_TRANSFER :

                state == ENTERING_BUCKET ? ANGLE_CLAW_TRANSFER :
                state == COUNTER_ROLLING ? ANGLE_CLAW_TRANSFER :
                state == CLAW_CLOSING ?     ANGLE_CLAW_SAMPLE :
                state == EXITING_BUCKET ?   ANGLE_CLAW_SAMPLE :

                state == LIFT_MOVING_TO_BASKET ? ANGLE_CLAW_SAMPLE :
                state == ARM_MOVING_TO_BASKET ?     ANGLE_CLAW_SAMPLE :
                state == AT_BASKET ?            ANGLE_CLAW_SAMPLE :
                state == FALLING_BASKET ?       ANGLE_CLAW_DROPPING_SAMPLE :
                state == BASKET_TO_STANDBY ? ANGLE_CLAW_TRANSFER :

                state == State.MOVING_TO_INTAKING_SPEC ?  ANGLE_CLAW_MOVING_TO_SPECIMEN :
                state == INTAKING_SPECIMEN ?        ANGLE_CLAW_INTAKING_SPECIMEN :
                state == GRABBING_SPECIMEN ?        ANGLE_CLAW_SPECIMEN :
                state == RAISING_SPECIMEN ?         ANGLE_CLAW_SPECIMEN :
                state == RAISED_TO_STANDBY ?        ANGLE_CLAW_SPECIMEN :
                state == STANDBY_TO_CHAMBER ?       ANGLE_CLAW_SPECIMEN :
                state == AT_CHAMBER ?               ANGLE_CLAW_SPECIMEN :
                state == RELEASING_SPECIMEN ? ANGLE_CLAW_TRANSFER :
                state == RELEASED_SPEC_TO_STANDBY ? ANGLE_CLAW_TRANSFER :

                ANGLE_CLAW_MOVING_TO_SPECIMEN
        );
    }

    public void setWristPitchingAngle(double t) {
        wristPitchingAngle = clip(abs(t), 0, 1);
    }

    public void preloadSpecimen() {
        state = AT_CHAMBER;
        claw.turnToAngle(ANGLE_CLAW_SPECIMEN);
    }

    public void goToBasket() {
        state = AT_BASKET;
        claw.turnToAngle(ANGLE_CLAW_SAMPLE);
    }

    public void preloadSample() {
        state = EXITING_BUCKET;
        claw.turnToAngle(ANGLE_CLAW_SAMPLE);
        lift.setTarget(sampleHeight);
    }

    // when does the intake need to move out of the way
    boolean requestingIntakeToMove() {
        return lift.getPosition() < HEIGHT_ABOVE_INTAKE && (
                        state.ordinal() >= WAITING_FOR_BUCKET.ordinal() &&
                        state.ordinal() <= STANDBY_TO_CHAMBER.ordinal()
        );
    }

    boolean readyToTransfer() {
        return state == State.STANDBY && !lift.isExtended();
    }

    public void setPosition(Position position) {

        switch (state) {

            case ENTERING_BUCKET:
            case COUNTER_ROLLING:
            case CLAW_CLOSING:
                break;

            case EXITING_BUCKET:
            case LIFT_MOVING_TO_BASKET:
            case ARM_MOVING_TO_BASKET:
            case AT_BASKET:
            case STANDBY:
            case RELEASED_SPEC_TO_STANDBY:
            case FALLING_BASKET:
            case BASKET_TO_STANDBY:
                lift.setTarget(
                        position == HIGH ?
                                lift.getTarget() >= HEIGHT_BASKET_HIGH ?
                                        lift.getTarget() + INCREMENT_REACH_ABOVE_BASKET :
                                        HEIGHT_BASKET_HIGH :
                        position == LOW ?
                                HEIGHT_BASKET_LOW :
                        0
                );
                break;

            case MOVING_TO_INTAKING_SPEC:
            case INTAKING_SPECIMEN:
                if (position != FLOOR) break;
                while (state != State.STANDBY) nextState();
                lift.setTarget(0);
                break;

            case GRABBING_SPECIMEN:
                if (position != FLOOR) break;
            case RAISING_SPECIMEN:
            case RAISED_TO_STANDBY:
            case STANDBY_TO_CHAMBER:
            case AT_CHAMBER:

                if (position == FLOOR) {
                    while (state != State.MOVING_TO_INTAKING_SPEC) nextState();
                    break;
                }

            case RELEASING_SPECIMEN:

                lift.setTarget(
                        position == HIGH ?
                                lift.getTarget() >= HEIGHT_CHAMBER_HIGH ?
                                        lift.getTarget() + INCREMENT_REACH_ABOVE_BASKET :
                                        HEIGHT_CHAMBER_HIGH :
                                lift.getTarget() >= HEIGHT_CHAMBER_LOW ?
                                        lift.getTarget() + INCREMENT_REACH_ABOVE_BASKET :
                                        HEIGHT_CHAMBER_LOW
                );

                break;
        }

    }

    public void nextState() {
        timer.reset();
        switch (state) {
            case RELEASED_SPEC_TO_STANDBY:
            case STANDBY:

                lift.setTarget(HEIGHT_INTAKING_SPECIMEN);
                state = WAITING_FOR_BUCKET;
                break;

            case CLAW_CLOSING:

                lift.setTarget(sampleHeight);
                state = EXITING_BUCKET;
                break;

            case GRABBING_SPECIMEN:
                lift.setTarget(topRung ? HEIGHT_CHAMBER_HIGH : HEIGHT_CHAMBER_LOW);
                state = state.next();
                break;

            case FALLING_BASKET:
            case RELEASING_SPECIMEN:
                lift.setTarget(0);
            case ENTERING_BUCKET:
            case COUNTER_ROLLING:
            case EXITING_BUCKET:
            case LIFT_MOVING_TO_BASKET:
            case ARM_MOVING_TO_BASKET:

            case WAITING_FOR_BUCKET:
            case MOVING_TO_INTAKING_SPEC:
            case INTAKING_SPECIMEN:
            case RAISING_SPECIMEN:
            case RAISED_TO_STANDBY:
            case STANDBY_TO_CHAMBER:

                state = state.next();
                break;

            case AT_BASKET:

                lastBasketPos = new EditablePose(dt.pose).toPose2d();
                sampleHeight = lift.getTarget() + PASSIVE_INCREMENT;
                state = state.next();
                break;

            case AT_CHAMBER:

                lastChamberY = dt.pose.position.y;
                topRung = lift.getTarget() > 0.5 * (HEIGHT_CHAMBER_HIGH + HEIGHT_CHAMBER_LOW);
                state = state.next();
                break;

            case BASKET_TO_STANDBY:

                state = State.STANDBY;
                break;
        }
    }

    public boolean hasSample() {
        return
                state == CLAW_CLOSING ||
                state == EXITING_BUCKET ||
                state == LIFT_MOVING_TO_BASKET ||
                state == ARM_MOVING_TO_BASKET ||
                state == AT_BASKET ||

                state == GRABBING_SPECIMEN ||
                state == RAISING_SPECIMEN ||
                state == RAISED_TO_STANDBY ||
                state == STANDBY_TO_CHAMBER ||
                state == AT_CHAMBER;
    }

    public boolean basketReady() {
        return state == AT_BASKET;
    }

    public boolean chamberReady() {
        return state == AT_CHAMBER;
    }

    public boolean intaking() {
        return state == INTAKING_SPECIMEN;
    }

    public boolean specGrabbed() {
        return state == RAISED_TO_STANDBY;
    }

    public void transfer() {
        if (state != State.STANDBY) return;
        state = ENTERING_BUCKET;
        timer.reset();
    }

    void printTelemetry() {
        String gameElement = state.ordinal() >= State.MOVING_TO_INTAKING_SPEC.ordinal() ? "has specimen" : "has sample";
        mTelemetry.addData("DEPOSIT", state + ", " + (hasSample() ? gameElement : "empty"));
        divider();
        lift.printTelemetry();
    }

    public static final class ArmPosition {

        public double arm, wrist;

        private ArmPosition(double arm, double wrist) {
            this.arm = arm;
            this.wrist = wrist;
        }
    }
}
