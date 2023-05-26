package org.firstinspires.ftc.teamcode.subsystems.pivot;

public class PivotValue {
    public volatile double pivotPosition;
    public volatile boolean shouldSensorWork;
    public volatile String string;
    public enum PivotEnum {
            RESET,
        TRANSFER,
        INTAKE_FRONT,
        INTAKE_BACK,
        AUTO_INTAKE_FRONT,
        AUTO_INTAKE_BACK,
        AUTO_FRONT,
        AUTO_HIGH_FRONT,
        AUTO_BACK,
        AUTO_HIGH_BACK,
        HIGH_BACK,
        BACK,
        HIGH_FRONT,
        FRONT,
        DROP_FRONT,
        DROP_BACK,
        AUTO_DROP_BACK, REST, GROUND,
        MANUAL // To use when the subsytem is going Manually - No Need to make PivotValue
    }
    public PivotEnum pivotEnum;

    public PivotValue(PivotEnum pivotEnum, double pivotPosition, boolean shouldSensorWork) {
        this.pivotEnum = pivotEnum;
        this.pivotPosition = pivotPosition;
        this.shouldSensorWork = shouldSensorWork;
        this.string = pivotEnum.toString();
    }
    public double getPivotPosition(){
        return pivotPosition;
    }
    public boolean getShouldSensorWork(){
        return shouldSensorWork;
    }
    public PivotEnum getEnum(){
        return pivotEnum;
    }
    public final boolean equals(Object other) {
        return this==other;
    }
    public String getString(){
        return string;
    }
}
