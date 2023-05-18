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
public class PivotClass extends SubsystemBase {
//    private static pos.PivotPos PivotPos;
    public final PIDFController controller;
    public boolean armAutomatic;
    public boolean shouldSensorWork = true;
//    public static  pos d;
//    public static class pos{
        public final static class PivotPos {
            public static final PivotPos RESET = new PivotPos(0);
            public static final PivotPos TELE_OP_START_POS= new PivotPos(-350);

            public static final PivotPos INTAKE_BACK= new PivotPos(0, true);
            public static final PivotPos BACK= new PivotPos(-455);
            public static final PivotPos HIGH_BACK= new PivotPos(-455);
            public static final PivotPos GROUND_BACK= new PivotPos(-480);
            public static final PivotPos DROP_BACK= new PivotPos(-700);

            public static final PivotPos INTAKE_FRONT= new PivotPos(-PivotPos.INTAKE_BACK.pivotPosition, true);
            public static final PivotPos FRONT= new PivotPos(-PivotClass.PivotPos.BACK.pivotPosition);
            public static final PivotPos HIGH_FRONT= new PivotPos(-PivotClass.PivotPos.HIGH_BACK.pivotPosition);
            public static final PivotPos GROUND_FRONT= new PivotPos(-PivotClass.PivotPos.GROUND_BACK.pivotPosition);
            public static final PivotPos DROP_FRONT= new PivotPos(-PivotClass.PivotPos.DROP_BACK.pivotPosition);


            public static final PivotPos AUTO_INTAKE_BACK= new PivotPos(-150, true);
            public static final PivotPos AUTO_BACK= new PivotPos(-233);
            public static final PivotPos AUTO_HIGH_BACK= new PivotPos(-150);
            public static final PivotPos AUTO_DROP_BACK= new PivotPos(-390);

            public static final PivotPos AUTO_INTAKE_FRONT= new PivotPos(-AUTO_INTAKE_BACK.pivotPosition, true);
            public static final PivotPos AUTO_FRONT= new PivotPos(-PivotClass.PivotPos.AUTO_BACK.pivotPosition);
            public static final PivotPos AUTO_HIGH_FRONT= new PivotPos(-PivotClass.PivotPos.AUTO_HIGH_BACK.pivotPosition);
            public static final PivotPos AUTO_DROP_FRONT= new PivotPos(-PivotClass.PivotPos.AUTO_DROP_BACK.pivotPosition);

            private final double pivotPosition;
            private final boolean shouldSensorWork;
//            public final DoubleValue pivotPosition;
//            public final Value<Boolean> shouldSensorWork;
            private PivotPos(double pivotPosition) {
                this.pivotPosition = pivotPosition;
                this.shouldSensorWork = false;
//                this.pivotPosition = new DoubleValue(pivotPosition);
//                this.shouldSensorWork = new Value<>(false);
//                this.shouldSensorWork = new Value(false);
            }
            private PivotPos(double pivotPosition, boolean shouldSensorWork) {
                this.pivotPosition = pivotPosition;
                this.shouldSensorWork = shouldSensorWork;
//                this.pivotPosition = new DoubleValue(pivotPosition);
//                this.shouldSensorWork = new Value<>(shouldSensorWork);
            }
        }
//    }
    public static PivotPos pivotPos = PivotPos.RESET;


    public final static double POWER = 0.93;
    public double encoderOffset = 0;
    Telemetry telemetry;
    public final NebulaMotor armMotor;

    public PivotClass(Telemetry tl, HardwareMap hw, boolean isEnabled) {
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
    public void dropArmTeleop(){//TODO:Check This
        if (PivotPos.FRONT.equals(pivotPos) || PivotPos.HIGH_FRONT.equals(pivotPos)) {
            setSetPoint(PivotPos.DROP_FRONT);
        } else if (PivotPos.BACK.equals(pivotPos) || PivotPos.HIGH_BACK.equals(pivotPos)) {
            setSetPoint(PivotPos.DROP_BACK);
        }
    }

    public void dropArmAuto(){//TODO:Check This
        if (PivotPos.AUTO_HIGH_BACK.equals(pivotPos) || PivotPos.AUTO_BACK.equals(pivotPos)) {
            setSetPoint(PivotPos.AUTO_DROP_BACK.pivotPosition, false);
        } else if (PivotPos.AUTO_HIGH_FRONT.equals(pivotPos) || PivotPos.AUTO_FRONT.equals(pivotPos)) {
            setSetPoint(PivotPos.AUTO_INTAKE_FRONT.pivotPosition, false);
        } else if (PivotPos.AUTO_INTAKE_FRONT.equals(pivotPos)) {
            setSetPoint(PivotPos.AUTO_INTAKE_FRONT.pivotPosition + 25, true);
        } else if (PivotPos.AUTO_INTAKE_BACK.equals(pivotPos)) {
            setSetPoint(PivotPos.AUTO_INTAKE_BACK.pivotPosition - 25, true);
        }
    }

    public void setSetPoint(PivotPos pos) {
        if(pivotPos.pivotPosition>NebulaConstants.Pivot.MAX_POSITION ||
            pivotPos.pivotPosition<NebulaConstants.Pivot.MIN_POSITION){
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
