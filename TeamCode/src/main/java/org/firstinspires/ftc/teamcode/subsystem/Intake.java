package org.firstinspires.ftc.teamcode.subsystem;

import static com.qualcomm.robotcore.util.Range.clip;
import static org.firstinspires.ftc.teamcode.control.vision.pipeline.Sample.BARNACLE;
import static org.firstinspires.ftc.teamcode.opmode.Auto.divider;
import static org.firstinspires.ftc.teamcode.opmode.Auto.mTelemetry;
import static org.firstinspires.ftc.teamcode.subsystem.Intake.State.ARM_ENTERING_BUCKET;
import static org.firstinspires.ftc.teamcode.subsystem.Intake.State.ARM_EXITING_BUCKET;
import static org.firstinspires.ftc.teamcode.subsystem.Intake.State.BUCKET_RETRACTING;
import static org.firstinspires.ftc.teamcode.subsystem.Intake.State.COUNTER_ROLLING;
import static org.firstinspires.ftc.teamcode.subsystem.Intake.State.SETTLING;
import static org.firstinspires.ftc.teamcode.subsystem.Intake.State.EJECTING_SAMPLE;
import static org.firstinspires.ftc.teamcode.subsystem.Intake.State.EXTENDO_RETRACTING;
import static org.firstinspires.ftc.teamcode.subsystem.Intake.State.EXTENDO_RE_EXTENDING;
import static org.firstinspires.ftc.teamcode.subsystem.Intake.State.STANDBY;
import static org.firstinspires.ftc.teamcode.control.vision.pipeline.Sample.BLUE;
import static org.firstinspires.ftc.teamcode.control.vision.pipeline.Sample.NEUTRAL;
import static org.firstinspires.ftc.teamcode.control.vision.pipeline.Sample.RED;
import static org.firstinspires.ftc.teamcode.subsystem.Intake.State.CLAW_CLOSING;
import static org.firstinspires.ftc.teamcode.subsystem.utility.cachedhardware.CachedSimpleServo.getAxon;
import static java.lang.Math.abs;
import static java.lang.Math.min;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.control.gainmatrix.HSV;
import org.firstinspires.ftc.teamcode.control.vision.pipeline.Sample;
import org.firstinspires.ftc.teamcode.subsystem.utility.cachedhardware.CachedMotorEx;
import org.firstinspires.ftc.teamcode.subsystem.utility.cachedhardware.CachedSimpleServo;
import org.firstinspires.ftc.teamcode.subsystem.utility.sensor.ColorSensor;

@Config
public final class Intake {

    public static double

            ANGLE_ARM_EXITING = 5,
            ANGLE_BUCKET_RETRACTED = 5,
            ANGLE_BUCKET_RETRACTED_OVER_BAR = 30,
            ANGLE_BUCKET_OVER_SUB_BAR = 80,
            ANGLE_AVOID_ARM = 60,
            ANGLE_BUCKET_INTAKING_NEAR = 140,
            ANGLE_BUCKET_INTAKING_FAR = 127,

            TIME_EJECTING = 0.5,
            TIME_DROP_TAG_ALONG_SAMPLE = 0.05,
            TIME_MAX_EXTEND_BEFORE_RE_RETRACT = 0.65,
            TIME_MAX_RETRACT_BEFORE_REATTEMPT = 2,
            TIME_BUCKET_RETRACT = 0,
            TIME_MAX_BUCKET_RETRACT = 0.75,
            TIME_BUCKET_SETTLING = 0,

            LENGTH_RE_RETRACT_TARGET = 150,
            LENGTH_TILT_BUCKET_OVER_BAR = 1,

            SPEED_EJECTING = -0.5,
            SPEED_HOLDING_BUT_EJECTING = -0.5,
            SPEED_HOLDING = 0.75,
            SPEED_ARM_ENTERING = 1,
            SPEED_COUNTER_ROLLING = -1,
            SPEED_CLAW_CLOSING = -1,
            SPEED_ARM_EXITING = -1,
            COLOR_SENSOR_GAIN = 1;

    /**
     * HSV value bound for intake bucket sample detection
     */
    public static HSV
            minRed = new HSV(
                    0,
                    0.25,
                    0.008
            ),
            maxRed = new HSV(
                    30,
                    0.75,
                    1
            ),
            minYellow = new HSV(
                    75,
                    0.6,
                    0.005
            ),
            maxYellow = new HSV(
                    96,
                    0.85,
                    1
            ),
            minBlue = new HSV(
                    215,
                    0.6,
                    0.008
            ),
            maxBlue = new HSV(
                    230,
                    0.9,
                    1
            ),
            minWhite = new HSV(
                    0,
                    0,
                    0.03
            ),
            maxWhite = new HSV(
                    360,
                    0.6,
                    1
            );

    /**
     * @return The {@link Sample} corresponding to the provided {@link HSV} as per the tuned value bounds
     */
    public static Sample hsvToSample(HSV hsv) {
        return
                hsv.between(minWhite, maxWhite) ? BARNACLE :
                hsv.between(minRed, maxRed) ? RED :
                hsv.between(minBlue, maxBlue) ? BLUE :
                hsv.between(minYellow, maxYellow) ? NEUTRAL :
                null;
    }

    public boolean retractBucketBeforeExtendo = true, specimenMode = false, doTransfer = true;

    private final CachedMotorEx roller;
    private double rollerSpeed, bucketAngle;

    private final ColorSensor colorSensor;
    private HSV hsv = new HSV();
    private Sample sample, oppSample = RED;

    private final CachedSimpleServo bucketR, bucketL;

    private void setBucket(double angle) {
        bucketR.turnToAngle(angle);
        bucketL.turnToAngle(angle);
    }
    
    private final TouchSensor bucketSensor;

    public final Extendo extendo;

    private Intake.State state = STANDBY;

    private final ElapsedTime timer = new ElapsedTime(), timeSinceIntaked = new ElapsedTime();

    enum State {
        EJECTING_SAMPLE,
        STANDBY,
        BUCKET_RETRACTING,
        EXTENDO_RE_EXTENDING,
        EXTENDO_RETRACTING,
        SETTLING,
        ARM_ENTERING_BUCKET,
        COUNTER_ROLLING,
        CLAW_CLOSING,
        ARM_EXITING_BUCKET,
    }

    public void setAlliance(boolean redAlliance) {
        oppSample = redAlliance ? BLUE : RED;
    }

    Intake(HardwareMap hardwareMap) {

        extendo = new Extendo(hardwareMap);

        bucketR = getAxon(hardwareMap, "bucket right").reversed();
        bucketL = getAxon(hardwareMap, "bucket left");

        roller = new CachedMotorEx(hardwareMap, "intake", Motor.GoBILDA.RPM_1620);
        roller.setInverted(true);

        colorSensor = new ColorSensor(hardwareMap, "bucket color", (float) COLOR_SENSOR_GAIN);

        bucketSensor = hardwareMap.get(TouchSensor.class, "bucket sensor");
    }

    void run(Deposit deposit) {

        double ANGLE_BUCKET_INTAKING = lerp(ANGLE_BUCKET_INTAKING_NEAR, ANGLE_BUCKET_INTAKING_FAR, extendo.getPosition() / Extendo.LENGTH_EXTENDED);

        double angleBucketRetracted = extendo.getPosition() > LENGTH_TILT_BUCKET_OVER_BAR ? ANGLE_BUCKET_RETRACTED_OVER_BAR : ANGLE_BUCKET_RETRACTED;

        double holdingSpeed = timeSinceIntaked.seconds() <= TIME_DROP_TAG_ALONG_SAMPLE ? SPEED_HOLDING_BUT_EJECTING : SPEED_HOLDING;

        switch (state) {

            case EJECTING_SAMPLE:

                setBucket(ANGLE_BUCKET_INTAKING);
                roller.set(SPEED_EJECTING);

                if (timer.seconds() >= TIME_EJECTING) state = STANDBY;
                else break;

            case STANDBY:

                if (rollerSpeed != 0) { // intaking, trigger held down

                    setBucket(lerp(ANGLE_BUCKET_OVER_SUB_BAR, ANGLE_BUCKET_INTAKING, abs(bucketAngle)));
                    roller.set(deposit.hasSample() ? min(rollerSpeed, 0) : rollerSpeed);
                    
                    colorSensor.update();
                    sample = hsvToSample(hsv = colorSensor.getHSV());

                    if (specimenMode && getSample() == NEUTRAL || getSample() == oppSample || getSample() == BARNACLE || hasSample() && deposit.hasSample()) {
                        ejectSample();
                        break;
                    }

                    if (hasSample() && rollerSpeed > 0 && doTransfer) transfer(sample);
                    else break;
                    
                } else { // retracted

                    setBucket(deposit.requestingIntakeToMove() ? ANGLE_AVOID_ARM : angleBucketRetracted);
                    roller.set(0);

                    break;

                }

            case BUCKET_RETRACTING:

                setBucket(angleBucketRetracted);
                roller.set(0);

                if (bucketSensor.isPressed() || timer.seconds() >= TIME_BUCKET_RETRACT || !retractBucketBeforeExtendo) {
                    state = EXTENDO_RETRACTING;
                    timer.reset();
                } else break;

            case EXTENDO_RE_EXTENDING:

                setBucket(angleBucketRetracted);
                roller.set(holdingSpeed);

                if (extendo.atPosition(LENGTH_RE_RETRACT_TARGET) || timer.seconds() > TIME_MAX_EXTEND_BEFORE_RE_RETRACT) {
                    state = EXTENDO_RETRACTING;
                    timer.reset();
                } else break;

            case EXTENDO_RETRACTING:

                setBucket(angleBucketRetracted);
                roller.set(holdingSpeed);
                extendo.setExtended(false);

                if (extendo.isExtended()) {
                    if (timer.seconds() >= TIME_MAX_RETRACT_BEFORE_REATTEMPT) {
                        state = EXTENDO_RE_EXTENDING;
                        extendo.setTarget(LENGTH_RE_RETRACT_TARGET);
                        timer.reset();
                    }
                    break;
                } else if ((bucketSensor.isPressed()|| timer.seconds() >= TIME_MAX_BUCKET_RETRACT) && deposit.readyToTransfer()) {
                    state = SETTLING;
                    timer.reset();
                } else break;

            case SETTLING:

                setBucket(angleBucketRetracted);
                roller.set(holdingSpeed);
                extendo.setExtended(false);

                if (timer.seconds() >= TIME_BUCKET_SETTLING) {
                    if (specimenMode) {
                        state = STANDBY;
                        timer.reset();
                        break;
                    } else {
                        state = ARM_ENTERING_BUCKET;
                        deposit.transfer();
                        timer.reset();
                    }
                } else break;

            case ARM_ENTERING_BUCKET:

                setBucket(angleBucketRetracted);
                roller.set(SPEED_ARM_ENTERING);
                extendo.setExtended(false);

                if (deposit.state == Deposit.State.COUNTER_ROLLING) {
                    state = COUNTER_ROLLING;
                    timer.reset();
                } else break;

            case COUNTER_ROLLING:

                setBucket(angleBucketRetracted);
                roller.set(SPEED_COUNTER_ROLLING);
                extendo.setExtended(false);

                if (deposit.state == Deposit.State.CLAW_CLOSING) {
                    state = CLAW_CLOSING;
                    sample = null;
                    timer.reset();
                } else break;

            case CLAW_CLOSING:

                setBucket(angleBucketRetracted);
                roller.set(SPEED_CLAW_CLOSING);
                extendo.setExtended(false);

                if (deposit.state == Deposit.State.EXITING_BUCKET) {
                    state = ARM_EXITING_BUCKET;
                    timer.reset();
                } else break;

            case ARM_EXITING_BUCKET:

                setBucket(ANGLE_ARM_EXITING);
                roller.set(SPEED_ARM_EXITING);
                extendo.setExtended(false);

                if (deposit.state != Deposit.State.EXITING_BUCKET) {
                    state = STANDBY;
                    timer.reset();
                } else break;
        }

        extendo.run();
    }

    public void transfer(Sample sample) {
        this.sample = sample;
        state = BUCKET_RETRACTING;
        timer.reset();
        timeSinceIntaked.reset();
    }

    public void ejectSample() {
        this.sample = null;
        state = EJECTING_SAMPLE;
        timer.reset();
    }

    /**
     * Interpolate between two values
     * @param start starting value, returned if t = 0
     * @param end ending value, returned if t = 1
     * @param t interpolation parameter, on the inclusive interval [0, 1]
     * @return the interpolated value t% between start and end, on the inclusive interval [start, end]
     */
    public static double lerp(double start, double end, double t) {
        return (1 - t) * start + t * end;
    }

    public boolean extendoRetracting() {
        return state == EXTENDO_RETRACTING;
    }

    public boolean hasSample() {
        return getSample() != null;
    }

    public Sample getSample() {
        return sample;
    }

    public void setRollerAndAngle(double power) {
        setRoller(power);
        setAngle(power);
    }

    public void setRoller(double power) {
        rollerSpeed = clip(power, -1, 1);
    }

    public boolean setAngle(double angle) {
        bucketAngle = clip(abs(angle), 0, 1);
        return bucketAngle == 1;
    }

    void printTelemetry() {
        mTelemetry.addData("INTAKE", state + ", " + (hasSample() ? getSample() + " sample" : "empty"));
        mTelemetry.addLine();
        mTelemetry.addData("Transfer", specimenMode ? "manually" : "on color detect");
        hsv.toTelemetry();
        divider();
        extendo.printTelemetry();
    }

}
