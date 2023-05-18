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
    public final PIDFController controller;
    public boolean armAutomatic;
    public boolean shouldSensorWork = true;

    public enum PivotPos {
        RESET(0),
        TELE_OP_START_POS(-350),

        INTAKE_BACK(0, true),
        BACK(-455),
        HIGH_BACK(-455),
        GROUND_BACK(-480),
        DROP_BACK(-700),

        INTAKE_FRONT(-PivotPos.INTAKE_BACK.pivotPosition, true),
        FRONT(-PivotPos.BACK.pivotPosition),
        HIGH_FRONT(-PivotPos.HIGH_BACK.pivotPosition),
        GROUND_FRONT(-PivotPos.GROUND_BACK.pivotPosition),
        DROP_FRONT(-PivotPos.DROP_BACK.pivotPosition),


        AUTO_INTAKE_BACK(-150, true),
        AUTO_BACK(-233),
        AUTO_HIGH_BACK(-150),
        AUTO_DROP_BACK(-390),

        AUTO_INTAKE_FRONT(-PivotPos.AUTO_INTAKE_BACK.pivotPosition, true),
        AUTO_FRONT(-PivotPos.AUTO_BACK.pivotPosition),
        AUTO_HIGH_FRONT(-PivotPos.AUTO_HIGH_BACK.pivotPosition),
        AUTO_DROP_FRONT(-PivotPos.AUTO_DROP_BACK.pivotPosition);

        public final double pivotPosition;
        public final boolean shouldSensorWork;
        PivotPos(double pivotPosition) {
            this.pivotPosition = pivotPosition;
            this.shouldSensorWork = false;
        }
        PivotPos(double pivotPosition, boolean shouldSensorWork) {
            this.pivotPosition = pivotPosition;
            this.shouldSensorWork = shouldSensorWork;
        }
    }
    public static PivotPos pivotPos = PivotPos.RESET;


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
        setSetPoint(PivotPos.RESET.pivotPosition - encoderOffset - 10, true);
        pivotPos = PivotPos.RESET;
    }
    public void moveIntakeBAuto() {
        armAutomatic = true;
        setSetPoint(PivotPos.AUTO_INTAKE_BACK);
    }
    public void dropArmTeleop(){
        switch (pivotPos){
            case FRONT:
            case HIGH_FRONT:
                setSetPoint(PivotPos.DROP_FRONT);
                break;
            case BACK:
            case HIGH_BACK:
                setSetPoint(PivotPos.DROP_BACK);
                break;
        }
    }

    public void dropArmAuto(){
        switch (pivotPos){
            case AUTO_HIGH_BACK:
            case AUTO_BACK:
                setSetPoint(PivotPos.AUTO_DROP_BACK.pivotPosition, false);
                break;
            case AUTO_HIGH_FRONT:
            case AUTO_FRONT:
                setSetPoint(PivotPos.AUTO_INTAKE_FRONT.pivotPosition,false);
                break;
            case AUTO_INTAKE_FRONT:
                setSetPoint(PivotPos.AUTO_INTAKE_FRONT.pivotPosition+25, true);
                break;
            case AUTO_INTAKE_BACK:
                setSetPoint(PivotPos.AUTO_INTAKE_BACK.pivotPosition-25, true);
                break;
        }
    }

    public void setSetPoint(PivotPos pos) {
        if(pos.pivotPosition>NebulaConstants.Pivot.MAX_POSITION ||
            pos.pivotPosition<NebulaConstants.Pivot.MIN_POSITION){
            armMotor.stop();
            return;
        }
        controller.setSetPoint(pos.pivotPosition + encoderOffset);
        pivotPos = pos;
        this.shouldSensorWork = pos.shouldSensorWork;
    }
    public void setSetPoint(double setPoint, boolean shouldSensorWork) {
        if(setPoint>NebulaConstants.Pivot.MAX_POSITION ||
            setPoint<NebulaConstants.Pivot.MIN_POSITION){
            armMotor.stop();
            return;
        }
        controller.setSetPoint(setPoint + encoderOffset);
        this.shouldSensorWork = shouldSensorWork;
    }

    //TODO: Test!
    public Command setSetPointCommand(double setPoint, boolean shouldSensorWork) {
        return new InstantCommand(()->{setSetPoint(setPoint, shouldSensorWork);});
    }
    public Command setSetPointCommand(PivotPos pos) {
        return new InstantCommand(()->{setSetPoint(pos);});
    }

    public void encoderReset() {
        armMotor.resetEncoder();
        telemetry.addLine("ARM RESET");
    }
    public double getSetPoint(){
        return controller.getSetPoint();
    }
}
