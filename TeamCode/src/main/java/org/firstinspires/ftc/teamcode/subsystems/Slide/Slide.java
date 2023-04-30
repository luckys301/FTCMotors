package org.firstinspires.ftc.teamcode.subsystems.Slide;

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
public class Slide extends SubsystemBase {
    public final Telemetry telemetry;
    public final MotorEx slideM1, slideM2;

    public PIDFController slideController;
    public boolean slideAutomatic;

    public static double CPR = 751.8;
    public double output = 0;

    public boolean lowBool = false;
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

        slideController = new PIDFController(NebulaConstants.Slide.slidePID.p,
            NebulaConstants.Slide.slidePID.i,
            NebulaConstants.Slide.slidePID.d,
            NebulaConstants.Slide.slidePID.f,
            getEncoderDistance(),
            getEncoderDistance());
        slideController.setTolerance(NebulaConstants.Slide.slideTolerance);

        this.telemetry = tl;
        slideAutomatic = false;
        liftPos = LiftPos.REST;
    }

    @Override
    public void periodic() {
        if (slideAutomatic) {
//            slideController.setF(NebulaConstants.Slide.slidePID.f * Math.cos(Math.toRadians(slideController.getSetPoint())));

            output = slideController.calculate(getEncoderDistance());
//            if (output >= 1) output = 1;
//            if (output <= -1) output = -1;

            setPower(output);//TODO: Probably shouldn't be like this
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
        telemetry.addData("     Lift1 Encoder: ", slideM1.getCurrentPosition());
        telemetry.addData("     List Pos:", getSetPoint());
    }

    public double getEncoderDistance() {
        return slideM1.getDistance();
    }


    public void setPower(double power) {
        slideM1.set(power);
        slideM2.set(-power);//Instead of putting -power, maybe reverse the motor
    }

    public void stopSlide() {
        slideM1.stopMotor();
        slideM2.stopMotor();
        slideController.setSetPoint(getEncoderDistance());

        slideAutomatic = false;
    }
    /****************************************************************************************/

    public void slideResting() {
        slideAutomatic = true;
        slideController.setSetPoint(LiftPos.REST.liftPosition);
        liftPos = LiftPos.REST;
    }

    public void resetEncoder() {
        slideM1.resetEncoder();
        slideM2.resetEncoder();
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
                slideController.setSetPoint(LiftPos.HIGH.liftPosition-740);
                break;
            case AUTO_MID:
                slideController.setSetPoint(LiftPos.AUTO_MID.liftPosition-290);
                break;
            case AUTO_HIGH:
                slideController.setSetPoint(LiftPos.AUTO_HIGH.liftPosition-650);
                break;
        }
    }
    public void setSetPoint(LiftPos pos) {//Might want to make this function go into the 2 variable one
        if(pos.liftPosition>NebulaConstants.Slide.MAX_POSITION ||
            pos.liftPosition<NebulaConstants.Slide.MIN_POSITION){
            slideM1.stopMotor();
            return;
        }
        slideController.setSetPoint(pos.liftPosition);
        liftPos = pos;
        this.lowBool = pos.lowBool;
    }
    public void setSetPoint(double setPoint, boolean lowBool) {
        if(setPoint>NebulaConstants.Slide.MAX_POSITION ||
            setPoint<NebulaConstants.Slide.MIN_POSITION){
            slideM1.stopMotor();
            return;
        }
        slideController.setSetPoint(setPoint);
        this.lowBool = lowBool;
    }

    //TODO: Test!
    public Command setSetPointCommand(double setPoint, boolean shouldSensorWork) {
        return new InstantCommand(()->{this.setSetPoint(setPoint, shouldSensorWork);});
    }
    public Command setSetPointCommand(LiftPos pos) {
        return new InstantCommand(()->{this.setSetPoint(pos);});
    }
    public double getSetPoint() {
        return slideController.getSetPoint();
    }
}