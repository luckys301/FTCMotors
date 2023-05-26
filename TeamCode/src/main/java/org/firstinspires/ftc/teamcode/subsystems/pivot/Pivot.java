package org.firstinspires.ftc.teamcode.subsystems.pivot;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.NebulaConstants;
import org.firstinspires.ftc.teamcode.util.misc.Util;
import org.firstinspires.ftc.teamcode.util.nebulaHardware.NebulaMotor;

import java.util.logging.Level;

@Config
public class Pivot extends SubsystemBase {
//    public static class PivotValue {
//        public volatile double pivotPosition;
//        public volatile boolean shouldSensorWork;
//        public volatile String string;
//        public enum PivotEnum {
//            RESET,
//            TRANSFER,
//            INTAKE_FRONT,
//            INTAKE_BACK,
//            AUTO_INTAKE_FRONT,
//            AUTO_INTAKE_BACK,
//            AUTO_FRONT,
//            AUTO_HIGH_FRONT,
//            AUTO_BACK,
//            AUTO_HIGH_BACK,
//            HIGH_BACK,
//            BACK,
//            HIGH_FRONT,
//            FRONT,
//            DROP_FRONT,
//            DROP_BACK,
//            AUTO_DROP_BACK, REST, GROUND,
//        }
//        public PivotEnum pivotEnum;
//
//        public PivotValue(PivotEnum pivotEnum, double pivotPosition, boolean shouldSensorWork) {
//            this.pivotEnum = pivotEnum;
//            this.pivotPosition = pivotPosition;
//            this.shouldSensorWork = shouldSensorWork;
//            this.string = pivotEnum.toString();
//        }
//        public double getPivotPosition(){
//            return pivotPosition;
//        }
//        public boolean getShouldSensorWork(){
//            return shouldSensorWork;
//        }
//        public PivotEnum getEnum(){
//            return pivotEnum;
//        }
//        public final boolean equals(Object other) {
//            return this==other;
//        }
//        public String getString(){
//            return string;
//        }
//    }
    public final PIDFController controller;
    public boolean armAutomatic;
    public boolean shouldSensorWork = true;

      PivotValue INTAKE_FRONT=new PivotValue(PivotValue.PivotEnum.INTAKE_FRONT,1, true);
      PivotValue INTAKE_BACK=new PivotValue(PivotValue.PivotEnum.INTAKE_BACK,5,false);
      PivotValue AUTO_INTAKE_BACK=new PivotValue(PivotValue.PivotEnum.AUTO_INTAKE_BACK,1, true);
      PivotValue AUTO_INTAKE_FRONT=new PivotValue(PivotValue.PivotEnum.AUTO_INTAKE_FRONT,5,false);
      PivotValue DROP_FRONT=new PivotValue(PivotValue.PivotEnum.DROP_FRONT,1, true);
    PivotValue DROP_BACK=new PivotValue(PivotValue.PivotEnum.DROP_FRONT,1, true);
    PivotValue AUTO_DROP_BACK=new PivotValue(PivotValue.PivotEnum.AUTO_DROP_BACK,5,false);
    PivotValue RESET=new PivotValue(PivotValue.PivotEnum.AUTO_DROP_BACK,5,false);

    public static PivotValue.PivotEnum pivotPos = PivotValue.PivotEnum.RESET; // HOlds CUrrent Pivot Position
//    public static String pivotPos = PivotValue.PivotEnum.RESET.toString(); // HOlds CUrrent Pivot Position


    public final static double POWER = 0.93;
    public double encoderOffset = 0;
    Telemetry telemetry;
    public final NebulaMotor armMotor;

    public Pivot(Telemetry tl, HardwareMap hw, boolean isEnabled) {
        armMotor = new NebulaMotor(hw, NebulaConstants.Pivot.pivotMName,
            NebulaConstants.Pivot.pivotType,
            NebulaConstants.Pivot.pivotDirection,
            NebulaConstants.Pivot.pivotIdleMode,
            isEnabled);
        armMotor.setDistancePerPulse(1);
        controller = new PIDFController(NebulaConstants.Pivot.pivotPID.p,
            NebulaConstants.Pivot.pivotPID.i,
            NebulaConstants.Pivot.pivotPID.d,
            NebulaConstants.Pivot.pivotPID.f,
            getEncoderDistance(),
            getEncoderDistance());
        controller.setTolerance(NebulaConstants.Pivot.pivotTolerance);
        this.telemetry = tl;
        armAutomatic = false;
    }

    @Override
    public void periodic() {
        if (armAutomatic) {
//            controller.setF(NebulaConstants.Pivot.pivotPID.f * Math.cos(Math.toRadians(controller.getSetPoint())));
            double output = (controller.calculate(getEncoderDistance()));
            telemetry.addData("CLaw Motor Output:", output);

            armMotor.setPower(output * POWER);
        }
        Util.logger(this, telemetry, Level.INFO, "Arm Encoder Pos: ", armMotor.getPosition());
        Util.logger(this, telemetry, Level.INFO, "Arm Set Point: ", getSetPoint());

    }


    public double getEncoderDistance() {
        return armMotor.getDistance() - encoderOffset;
    }
    public double getAngle() {
        return getEncoderDistance();
    }

//    public void stopArm() {
//        armMotor.stopMotor();
//        armAutomatic = false;
//    }

    public void moveInitializationPosition() {
        armAutomatic = true;
        setSetPoint(RESET.pivotPosition - encoderOffset - 10, true);
        pivotPos = PivotValue.PivotEnum.RESET;
    }
    public void moveIntakeBAuto() {
        armAutomatic = true;
        setSetPointCommand(AUTO_INTAKE_BACK);
    }
    public void dropArmTeleop(){
        switch (pivotPos){
            case FRONT:
            case HIGH_FRONT:
                setSetPointCommand(DROP_FRONT);
                break;
            case BACK:
            case HIGH_BACK:
                setSetPointCommand(DROP_BACK);
                break;
        }
    }

    public void dropArmAuto(){
        switch (pivotPos){
            case AUTO_HIGH_BACK:
            case AUTO_BACK:
                setSetPoint(AUTO_DROP_BACK.pivotPosition, false);
                break;
            case AUTO_HIGH_FRONT:
            case AUTO_FRONT:
                setSetPoint(AUTO_INTAKE_FRONT.pivotPosition,false);
                break;
            case AUTO_INTAKE_FRONT:
                setSetPoint(AUTO_INTAKE_FRONT.pivotPosition+25, true);
                break;
            case AUTO_INTAKE_BACK:
                setSetPoint(AUTO_INTAKE_BACK.pivotPosition-25, true);
                break;
        }
    }

//    public void setSetPoint(PivotValue pos) {
//        if(pos.pivotPosition>NebulaConstants.Pivot.MAX_POSITION ||
//            pos.pivotPosition<NebulaConstants.Pivot.MIN_POSITION){
//            armMotor.stop();
//            return;
//        }
//        controller.setSetPoint(pos.pivotPosition + encoderOffset);
//        pivotPos = pos.getEnum();
//        this.shouldSensorWork = pos.shouldSensorWork;
//    }
    public void setSetPoint(double setPoint, boolean shouldSensorWork) {
        if(setPoint>NebulaConstants.Pivot.MAX_POSITION ||
            setPoint<NebulaConstants.Pivot.MIN_POSITION){
            armMotor.stop();
            return;
        }
//        pivotPos = setPoint;
        controller.setSetPoint(setPoint + encoderOffset);
        this.shouldSensorWork = shouldSensorWork;
    }

    //TODO: Test!
    public Command setSetPointCommand(double setPoint, boolean shouldSensorWork) {
        return new InstantCommand(()->{setSetPoint(setPoint, shouldSensorWork);});
    }
    public Command setSetPointCommand(PivotValue pos) {
        return new InstantCommand(()->{setSetPoint(pos.pivotPosition, pos.shouldSensorWork);});
    }

    public void encoderReset() {
        armMotor.resetEncoder();
        telemetry.addLine("ARM RESET");
    }
    public double getSetPoint(){
        return controller.getSetPoint();
    }
}
