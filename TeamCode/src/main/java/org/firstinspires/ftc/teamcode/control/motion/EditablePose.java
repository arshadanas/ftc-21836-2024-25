package org.firstinspires.ftc.teamcode.control.motion;

import static java.lang.Math.atan2;
import static java.lang.Math.hypot;
import static java.lang.Math.toDegrees;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;

public final class EditablePose {

    public double x, y, heading;

    public EditablePose(double x, double y, double heading) {
        this.x = x;
        this.y = y;
        this.heading = heading;
    }

    public EditablePose(Pose2d pose) {
        this(pose.position.x, pose.position.y, pose.heading.log());
    }

    public EditablePose clone() {
        return new EditablePose(x, y, heading);
    }

    public Pose2d toPose2d() {
        return new Pose2d(x, y, heading);
    }

    public Vector2d toVector2d() {
        return new Vector2d(x, y);
    }

    public double angleTo(EditablePose target) {
        return atan2(target.y - this.y, target.x - this.x);
    }

    public double distTo(EditablePose target) {
        return hypot(target.y - this.y, target.x - this.x);
    }

    public String toString() {
        return x + ", " + y + ", " + toDegrees(heading) + " deg";
    }
}
