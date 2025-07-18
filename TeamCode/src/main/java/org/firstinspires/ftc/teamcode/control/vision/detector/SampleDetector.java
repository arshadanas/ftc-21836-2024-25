package org.firstinspires.ftc.teamcode.control.vision.detector;

import static java.lang.Math.atan2;
import static java.lang.Math.tan;
import static java.lang.Math.toRadians;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.control.motion.EditablePose;

import java.util.ArrayList;
import java.util.List;
import java.util.Objects;

@Config
public class SampleDetector {
    
    private final Limelight3A limelight;

    public final List<LLResultTypes.DetectorResult> detections = new ArrayList<>();

    public enum Pipeline {
        YELLOW_BLUE,
        YELLOW_RED,
        BLUE,
        RED,
    }

    public double
            xDegFromLens = 0,
            yDegFromLens = 0,
            xDistFromLens = 0,
            yDistFromLens = 0;

    public EditablePose offsetToSample = new EditablePose(0, 0, 0);

    public SampleDetector(HardwareMap hardwareMap) {
        this.limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(10);
        limelight.stop();
        limelight.start();
    }

    public void setPipeline(Pipeline pipeline) {
        limelight.pipelineSwitch(pipeline.ordinal());
    }

    /**
     * @return true if detections has at least one detection, false if no detections
     */
    public boolean run() {
        boolean wasEmpty = detections.isEmpty();
        detections.clear();

        LLResult result = limelight.getLatestResult();
        if (result == null) return false;

        List<LLResultTypes.DetectorResult> llResults = result.getDetectorResults();
        if (llResults == null) return false;

        llResults.removeIf(Objects::isNull);
        if (llResults.isEmpty()) return false;

        detections.addAll(llResults);

        if (wasEmpty) limelight.captureSnapshot("detected");

        double
                xDistCenterToLens = 4.4,
                yDistCenterToLens = 4.5,
                yDegLensFromFlatOnFloor = 60,
                inchesLensFromFloor = 16;

        xDegFromLens = detections.get(0).getTargetXDegrees();
        yDegFromLens = detections.get(0).getTargetYDegrees();

        yDistFromLens = inchesLensFromFloor * tan(toRadians(yDegLensFromFlatOnFloor + yDegFromLens));
        xDistFromLens = tan(toRadians(xDegFromLens)) * yDistFromLens;

        offsetToSample.y = yDistFromLens + yDistCenterToLens;
        offsetToSample.x = xDistFromLens + xDistCenterToLens;
        offsetToSample.heading = atan2(offsetToSample.x, offsetToSample.y);

        return true;
    }

}
