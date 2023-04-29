package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.NebulaConstants;

@Config
public class  Slide extends SubsystemBase {
    private final Telemetry telemetry;
    private final MotorEx slideM1,
        slideM2;

    private PIDFController controller;
    private boolean slideAutomatic;

    public static double CPR = 751.8;
    private final double encoderOffset = 0;
    double output = 0;

    public static boolean lowBool = false;
    public enum LiftPos{
        REST(-14,true),
        GROUND(30, true), LOW(315), MID(738), HIGH(1240),
        AUTO_MID(995), AUTO_HIGH(1260),

        FIVE(260), FOUR(210), THREE(150), TWO(100), ONE(0);

        public final double liftPosition;
        public final boolean lowBool;
        LiftPos(double liftPosition) {
            this.liftPosition = liftPosition;
            this.lowBool = false;
        }
        LiftPos(double liftPosition, boolean lowBool) {
            this.liftPosition = liftPosition;
            this.lowBool = lowBool;
        }
    }
    LiftPos liftPos;


    public Slide( Telemetry tl, HardwareMap hw) {

        slideM1 = new MotorEx(hw, "lift");
        slideM2 = new MotorEx(hw, "lift2");

        slideM1.setDistancePerPulse(360 / CPR);
        slideM2.setDistancePerPulse(360 / CPR);

        controller = new PIDFController(NebulaConstants.Slide.slidePID.p,
            NebulaConstants.Slide.slidePID.i,
            NebulaConstants.Slide.slidePID.d,
            NebulaConstants.Slide.slidePID.f,
            getEncoderDistance(),
            getEncoderDistance());
        controller.setTolerance(NebulaConstants.Slide.slideTolerance);

        this.telemetry = tl;
        slideAutomatic = false;
        liftPos = LiftPos.REST;
        setOffset();
    }

    @Override
    public void periodic() {
        if (slideAutomatic) {
            controller.setF(NebulaConstants.Slide.slidePID.f * Math.cos(Math.toRadians(controller.getSetPoint())));

            output = controller.calculate(getEncoderDistance());
//            if (output >= 1) output = 1;
//            if (output <= -1) output = -1;

            slideM1.set(output);
            slideM2.set(-output);//TODO: Probably shouldn't be like this
//            if (lowBool) {
//                slideM1.set(output * LOW_POWER);
//                slideM2.set(output * LOW_POWER);
//            }
//            else {
//                slideM1.set(output * POWER);
//                slideM2.set(output * POWER);
//            }
        }
        telemetry.addLine("Slide - ");
        telemetry.addData("     Lift Motor Output:", output);
//        telemetry.addData("     Lift Motor 1 Power", slideM1.getVelocity());
//        telemetry.addData("     Lift Motor 2 Power:", slideM2.getVelocity());

        telemetry.addData("     Lift1 Encoder: ", slideM1.getCurrentPosition());
        telemetry.addData("     Lift2 Encoder: ", slideM2.getCurrentPosition());
        telemetry.addData("     List Pos:", liftPos);
    }

    private double getEncoderDistance() {
        return slideM1.getDistance() - encoderOffset;
    }


    public void setPower(double power) {
        slideM1.set(power);
        slideM2.set(power);
    }

    public void stopSlide() {
        slideM1.stopMotor();
        controller.setSetPoint(getEncoderDistance());
        slideM2.stopMotor();
        slideAutomatic = false;
    }

//    public double getAngle() {return getEncoderDistance();}

    /****************************************************************************************/

    public void slideResting() {
        slideAutomatic = true;
        controller.setSetPoint(LiftPos.REST.liftPosition);
        liftPos = LiftPos.REST;
    }

    public void encoderRecenter() {
        slideM1.resetEncoder();
        slideM2.resetEncoder();
    }


    public void setOffset() {
//        resetEncoder();
        controller.setSetPoint(getEncoderDistance());
    }
    public boolean isSlideAutomatic(){
        return slideAutomatic;
    }

    public void dropSlide(){
        switch (liftPos){
//            case LOW:
//                upController.setSetPoint(LOW_POS+350);
//                break;
//            case MID:
//                upController.setSetPoint(MID_POS+200);
//                break;
            case HIGH:
                controller.setSetPoint(LiftPos.HIGH.liftPosition-740);
                break;
            case AUTO_MID:
                controller.setSetPoint(LiftPos.AUTO_MID.liftPosition-290);
                break;
            case AUTO_HIGH:
                controller.setSetPoint(LiftPos.AUTO_HIGH.liftPosition-650);
                break;
        }
    }
    public void setSetPoint(LiftPos pos) {
        controller.setSetPoint(pos.liftPosition + encoderOffset);
        liftPos = pos;
        this.lowBool = pos.lowBool;
    }
    public void setSetPoint(double setPoint, boolean lowBool) {
        controller.setSetPoint(setPoint + encoderOffset);
        this.lowBool = lowBool;
    }

    //TODO: Test!
    public Command setSetPointCommand(double setPoint, boolean shouldSensorWork) {
        return new InstantCommand(()->{this.setSetPoint(setPoint, shouldSensorWork);});
    }
    public Command setSetPointCommand(LiftPos pos) {
        return new InstantCommand(()->{this.setSetPoint(pos);});
    }
    public double getPosition() {
        return controller.getSetPoint();
    }
}