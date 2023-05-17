package org.firstinspires.ftc.teamcode.util.trajectorysequence.container;

import com.acmerobotics.roadrunner.geometry.Pose2d;

public class Pose2dContainer extends PathSegment {
    public volatile double x = 0;
    public volatile double y = 0;
    public volatile double heading = 0;
    public Pose2dContainer(double x, double y, double headingDegrees) {
        this.x = x;
        this.y = y;
        this.heading = headingDegrees;
    }

    /**
     *
     * @return Pose2d with heading converted to radians
     */
    public Pose2d getPose() {
        return new Pose2d(
                x,
                y,
                Math.toRadians(heading)
        );
    }
}
