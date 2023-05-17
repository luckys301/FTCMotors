package org.firstinspires.ftc.teamcode.util.trajectorysequence.container;

public class LineToConstantHeading extends PathSegment {
    public volatile double x, y;
    public LineToConstantHeading(double x, double y) {
        this.x = x;
        this.y = y;
    }
}
